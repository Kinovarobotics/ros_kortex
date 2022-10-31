/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "kortex_driver/non-generated/joint_trajectory_action_server.h"
#include <sstream>
#include <fstream>
#include <thread> // for gcc 11 support

namespace {
    constexpr float STARTING_POINT_ARBITRARY_DURATION = 0.5f;
}

JointTrajectoryActionServer::JointTrajectoryActionServer(const std::string& server_name, ros::NodeHandle& nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic, Kinova::Api::ControlConfig::ControlConfigClient* control_config, bool use_hard_limits):
    m_server_name(server_name),
    m_node_handle(nh),
    m_server(nh, server_name, boost::bind(&JointTrajectoryActionServer::goal_received_callback, this, _1), boost::bind(&JointTrajectoryActionServer::preempt_received_callback, this, _1), false),
    m_base(base),
    m_base_cyclic(base_cyclic),
    m_control_config(control_config),
    m_server_state(ActionServerState::INITIALIZING),
    m_use_hard_limits(use_hard_limits)
{
    // Get the ROS params
    if (!ros::param::get("~default_goal_time_tolerance", m_default_goal_time_tolerance))
    {
        ROS_WARN("Parameter default_goal_time_tolerance was not specified; assuming 0.5 as default value.");
        m_default_goal_time_tolerance = 0.5;
    }
    if (!ros::param::get("~default_goal_tolerance", m_default_goal_tolerance))
    {
        ROS_WARN("Parameter default_goal_tolerance was not specified; assuming 0.005 as default value.");
        m_default_goal_tolerance = 0.005;
    }
    if (!ros::param::get("~prefix", m_prefix))
    {
        std::string error_string = "Prefix name was not specified in the launch file, shutting down the node...";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }
    if (!ros::param::get("~joint_names", m_joint_names))
    {
        std::string error_string = "Parameter joint_names was not specified";
        ROS_ERROR("%s", error_string.c_str());
        throw new std::runtime_error(error_string);
    }

    // Get the hard limits
    m_hard_limits = m_control_config->GetKinematicHardLimits();
    
    // Subscribe to the arm's Action Notifications
    m_sub_action_notif_handle = m_base->OnNotificationActionTopic(std::bind(&JointTrajectoryActionServer::action_notif_callback, this, std::placeholders::_1), Kinova::Api::Common::NotificationOptions());

    // Ready to receive goal
    m_server.start();
    set_server_state(ActionServerState::IDLE);
}

JointTrajectoryActionServer::~JointTrajectoryActionServer()
{
    m_base->Unsubscribe(m_sub_action_notif_handle);
}

void JointTrajectoryActionServer::goal_received_callback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle new_goal_handle)
{
    ROS_INFO("New goal received.");
    if (!is_goal_acceptable(new_goal_handle))
    {
        ROS_ERROR("Joint Trajectory Goal is rejected.");
        new_goal_handle.setRejected();
        return;
    }

    if (m_server_state != ActionServerState::IDLE)
    {
        ROS_WARN("There is already an active goal. It is being cancelled.");
        // We have to call Stop after having received the ACTION_START notification from the arm
        stop_all_movement();
    }

    // Accept the goal
    ROS_INFO("Joint Trajectory Goal is accepted.");
    m_goal = new_goal_handle;
    m_goal.setAccepted();

    // Construct the Protobuf object trajectory
    Kinova::Api::Base::WaypointList proto_trajectory;

    // Uncomment for logging purposes
    // std::ofstream myfile;
    // myfile.open ("/tmp/moveit_trajectory.csv");
    // for (unsigned int i = 0; i < new_goal_handle.getGoal()->trajectory.points.size(); i++)
    // {
    //     myfile << new_goal_handle.getGoal()->trajectory.points[i].time_from_start.toSec() << ",";
    //     for (auto position : new_goal_handle.getGoal()->trajectory.points[i].positions)
    //     {
    //         myfile << position << ",";
    //     }
    //     myfile << "\n";
    // }
    // myfile.close();

    // Copy the trajectory points from the ROS structure to the Protobuf structure
    for (unsigned int i = 0; i < new_goal_handle.getGoal()->trajectory.points.size(); i++)
    {
        // Create the waypoint
        Kinova::Api::Base::Waypoint* proto_waypoint = proto_trajectory.add_waypoints();
        proto_waypoint->set_name("waypoint_" + std::to_string(i));
        auto angular_waypoint = proto_waypoint->mutable_angular_waypoint();
        
        // Calculate the duration of this waypoint
        const auto traj_point = new_goal_handle.getGoal()->trajectory.points.at(i);
        float waypoint_duration;
        // FIXME We have to include the starting point for the trajectory as the first waypoint AND since optimal durations are not
        //       supported yet, we also have to give it a non-zero duration. Putting something too small gives errors so I've put 
        //       an arbitrary value of 500ms here for now.
        if (i == 0)
        {
            waypoint_duration = STARTING_POINT_ARBITRARY_DURATION;
        }
        else
        {
            auto previous_wp_duration = new_goal_handle.getGoal()->trajectory.points.at(i-1).time_from_start.toSec();
            waypoint_duration = traj_point.time_from_start.toSec() - previous_wp_duration;
            // Very small durations are rejected so we cap them
            if (waypoint_duration < 0.01)
            {
                waypoint_duration = 0.01f;
            }
        }
        
        // Fill the waypoint
        for (auto position : traj_point.positions)
        {
            angular_waypoint->add_angles(KortexMathUtil::toDeg(position));
        }
        angular_waypoint->set_duration(waypoint_duration);
    }

    // If the hard limits are used for the trajectory
    if (m_use_hard_limits)
    {
        // Get the soft limits
        m_soft_limits = getAngularTrajectorySoftLimits();

        // Set the soft limits to hard limits
        setAngularTrajectorySoftLimitsToMax();
    }

    try
    {
        // Validate the waypoints and reject the goal if they fail validation
        auto report = m_base->ValidateWaypointList(proto_trajectory);

        if (report.trajectory_error_report().trajectory_error_elements_size() > 0)
        {
            ROS_ERROR("Joint Trajectory failed validation in the arm.");
            // Go through report and print errors
            for (unsigned int i = 0; i < report.trajectory_error_report().trajectory_error_elements_size(); i++)
            {
                ROS_ERROR("Error %i : %s", i+1, report.trajectory_error_report().trajectory_error_elements(i).message().c_str());
            }
            setAngularTrajectorySoftLimits(m_soft_limits);
            new_goal_handle.setAborted();
            return;
        }
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_ERROR("Kortex exception while validating the trajectory");
        ROS_ERROR("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_ERROR("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_ERROR("Error description: %s\n", ex.what());
        m_goal.setAborted();
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_ERROR("Runtime exception detected while sending the trajectory");
        ROS_ERROR("%s", ex_runtime.what());
        m_goal.setAborted();
    }
    catch (std::future_error& ex_future)
    {
        ROS_ERROR("Future exception detected while getting feedback");
        ROS_ERROR("%s", ex_future.what());
        m_goal.setAborted();
    }
    
    try
    {
        // Make sure to clear the faults before moving the robot
        m_base->ClearFaults();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Send the trajectory to the robot
        m_base->ExecuteWaypointTrajectory(proto_trajectory);
        set_server_state(ActionServerState::PRE_PROCESSING_PENDING);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_ERROR("Kortex exception while sending the trajectory");
        ROS_ERROR("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_ERROR("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_ERROR("Error description: %s\n", ex.what());
        setAngularTrajectorySoftLimits(m_soft_limits);
        m_goal.setAborted();
    }
    catch (std::runtime_error& ex_runtime)
    {
        ROS_ERROR("Runtime exception detected while sending the trajectory");
        ROS_ERROR("%s", ex_runtime.what());
        setAngularTrajectorySoftLimits(m_soft_limits);
        m_goal.setAborted();
    }
    catch (std::future_error& ex_future)
    {
        ROS_ERROR("Future exception detected while getting feedback");
        ROS_ERROR("%s", ex_future.what());
        setAngularTrajectorySoftLimits(m_soft_limits);
        m_goal.setAborted();
    }
}

// Called in a separate thread when a preempt request comes in from the Action Client
void JointTrajectoryActionServer::preempt_received_callback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle)
{
    if (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS)
    {
        stop_all_movement();
    }
}

// Called in a separate thread when a notification comes in
void JointTrajectoryActionServer::action_notif_callback(Kinova::Api::Base::ActionNotification notif)
{
    Kinova::Api::Base::ActionEvent event = notif.action_event();
    Kinova::Api::Base::ActionHandle handle = notif.handle();
    Kinova::Api::Base::ActionType type = handle.action_type();
    ROS_DEBUG("Action notification received of type %s", Kinova::Api::Base::ActionEvent_Name(event).c_str());
    std::lock_guard<std::mutex> guard(m_action_notification_thread_lock);

    control_msgs::FollowJointTrajectoryResult result;
    std::ostringstream oss;

    if (type == Kinova::Api::Base::ActionType::EXECUTE_WAYPOINT_LIST)
    {
        switch (event)
        {
        // The pre-processing is starting in the arm
        case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_START:
            // It should be starting
            if (m_server_state == ActionServerState::PRE_PROCESSING_PENDING)
            {
                ROS_INFO("Preprocessing has started in the arm.");
                set_server_state(ActionServerState::PRE_PROCESSING_IN_PROGRESS);
            }
            // We should not have received that
            else
            {
                ROS_DEBUG("Notification mismatch : received ACTION_PREPROCESS_START but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The pre-processing has ended successfully in the arm
        case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_END:
            // It was ongoing and now it ended
            if (m_server_state == ActionServerState::PRE_PROCESSING_PENDING ||
                m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS)
            {
                ROS_INFO("Preprocessing has finished in the arm and goal has been accepted.");
                set_server_state(ActionServerState::TRAJECTORY_EXECUTION_PENDING);
            }
            // FIXME KOR-3563 Sometimes the notifications arrive in the wrong order so it is possible to receive
            // a ACTION_PREPROCESS_END notification after the ACTION_START
            // When this bug will be fixed this else if can be removed
            else if (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS)
            {
                ROS_DEBUG("Notification order mismatch : We received the ACTION_PREPROCESS_END after the ACTION_START");
                break;
            }
            // We should not have received that
            else
            {
                ROS_DEBUG("Notification mismatch : received ACTION_PREPROCESS_END but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The pre-processing has failed in the arm
        case Kinova::Api::Base::ActionEvent::ACTION_PREPROCESS_ABORT:
            // It was ongoing and now it ended (and failed)
            if ((m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS))
            {
                ROS_ERROR("Preprocessing has finished in the arm and goal has been rejected. Fetching the error report from the arm...");

                result.error_code = result.INVALID_GOAL;

                // Get the error report and show errors here
                Kinova::Api::Base::TrajectoryErrorReport report = m_base->GetTrajectoryErrorReport();
                oss << "Error report has been fetched and error elements are listed below : " << std::endl;
                int i = 1;
                for (auto error_element : report.trajectory_error_elements())
                {
                    oss << "-----------------------------" << std::endl;
                    oss << "Error #" << i << std::endl;
                    oss << "Type : " << Kinova::Api::Base::TrajectoryErrorType_Name(error_element.error_type()) << std::endl;
                    oss << "Actuator : " << error_element.index()+1 << std::endl;
                    oss << "Erroneous value is " << error_element.error_value() << " but minimum permitted is " << error_element.min_value() << " and maximum permitted is " << error_element.max_value() << std::endl;
                    if (error_element.message() != "")
                    {
                        oss << "Additional message is : " << error_element.message() << std::endl;
                    }
                    oss << "-----------------------------" << std::endl;

                    i++;
                }

                ROS_ERROR("%s", oss.str().c_str());

                result.error_string = oss.str();
                m_goal.setAborted(result);

                set_server_state(ActionServerState::IDLE);
            }
            // We should not have received that
            else
            {
                ROS_DEBUG("Notification mismatch : received ACTION_PREPROCESS_ABORT but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The arm is starting to move
        case Kinova::Api::Base::ActionEvent::ACTION_START:
            // The preprocessing was done and the goal is still active (not preempted)
            if ((m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING ||
                 m_server_state == ActionServerState::PRE_PROCESSING_IN_PROGRESS) && // FIXME KOR-3563 this happens if we received a ACTION_START before a ACTION_PREPROCESS_END
                 m_goal.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
            {
                ROS_INFO("Trajectory has started.");
                set_server_state(ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS);
                // Remember when the trajectory started
                m_trajectory_start_time = std::chrono::system_clock::now();
            }
            // The preprocessing was done but the goal put to "PREEMPTING" by the client while preprocessing
            // The stop_all_movement() call will trigger a ACTION_ABORT notification
            else if ((m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING) &&
                      m_goal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
            {
                ROS_INFO("Trajectory has started but goal was cancelled : stopping all movement.");
                stop_all_movement();
            }
            // We should not have received that
            else
            {
                ROS_DEBUG("Notification mismatch : received ACTION_START but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        case Kinova::Api::Base::ActionEvent::ACTION_FEEDBACK:
        {
            // debug trace to indicate we've reached waypoints
            for (unsigned int i = 0; i < notif.trajectory_info_size(); i++)
            {
                auto info = notif.trajectory_info(i);
                if (info.trajectory_info_type() == Kinova::Api::Base::TrajectoryInfoType::WAYPOINT_REACHED)
                {
                    ROS_DEBUG("Waypoint %d reached", info.waypoint_index());
                }
            }
            break;
        }   

        // The action was started in the arm, but it aborted
        case Kinova::Api::Base::ActionEvent::ACTION_ABORT:
            // The goal is still active, but we received a ABORT before starting, or during execution
            if (m_goal.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE &&
                (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS ||
                 m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING))
            {
                ROS_ERROR("Trajectory has been aborted.");

                result.error_code = result.PATH_TOLERANCE_VIOLATED;
                oss << "Trajectory execution failed in the arm with sub error code " << notif.abort_details() << std::endl;
                if (notif.abort_details() == Kinova::Api::SubErrorCodes::CONTROL_WRONG_STARTING_POINT)
                {
                    oss << "The starting point for the trajectory did not match the actual commanded joint angles." << std::endl;
                }
                else if (notif.abort_details() == Kinova::Api::SubErrorCodes::CONTROL_MANUAL_STOP)
                {
                    oss << "The speed while executing the trajectory was too damn high and caused the robot to stop." << std::endl;
                }
                result.error_string = oss.str();
                m_goal.setAborted(result);

                ROS_ERROR("%s", oss.str().c_str());
                set_server_state(ActionServerState::IDLE);
            }
            // The goal was cancelled and we received a ACTION_ABORT : this means the trajectory was cancelled successfully in the arm
            else if  (m_goal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING &&
                     (m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS ||
                      m_server_state == ActionServerState::TRAJECTORY_EXECUTION_PENDING))
            {
                ROS_INFO("Trajectory has been cancelled successfully in the arm.");
                m_goal.setCanceled();
                set_server_state(ActionServerState::IDLE);
            }
            // We should not have received that
            else
            {
                ROS_DEBUG("Notification mismatch : received ACTION_ABORT but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;

        // The trajectory just ended
        case Kinova::Api::Base::ActionEvent::ACTION_END:
        {
            // The trajectory was ongoing
            if ((m_server_state == ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS))
            {
                ROS_INFO("Trajectory has finished in the arm.");
                m_trajectory_end_time = std::chrono::system_clock::now();
                // When going at full speed we have to stabilize a bit before checking the final position
                if (m_use_hard_limits)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                bool is_tolerance_respected = is_goal_tolerance_respected(true, true);
                if (is_tolerance_respected)
                {
                    result.error_code = result.SUCCESSFUL;
                    ROS_INFO("Trajectory execution succeeded.");
                    m_goal.setSucceeded(result);
                }
                else
                {
                    result.error_code = result.PATH_TOLERANCE_VIOLATED;
                    oss << "After validation, trajectory execution failed in the arm with sub error code " << Kinova::Api::SubErrorCodes_Name(notif.abort_details());
                    result.error_string = oss.str();

                    ROS_ERROR("%s", oss.str().c_str());
                    m_goal.setAborted(result);
                }
                set_server_state(ActionServerState::IDLE);
            }
            // We should not have received that
            else
            {
                ROS_DEBUG("Notification mismatch : received ACTION_END but we are in %s", actionServerStateNames[int(m_server_state)]);
            }
            break;
        }

        case Kinova::Api::Base::ActionEvent::ACTION_PAUSE:
            ROS_DEBUG("Action pause event was just received and this should never happen.");
            break;

        default:
            ROS_DEBUG("Unknown action event was just received and this should never happen.");
            break;
        }
    }
    // Wrong action type. Rejecting the notification. Action server state unchanged.
    else
    {
        return;
    }

    oss.flush();
}

bool JointTrajectoryActionServer::is_goal_acceptable(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle)
{
    // First check if goal is valid
    if (!goal_handle.isValid())
    {
        ROS_ERROR("Invalid goal.");
        return false;
    }

    // Retrieve the goal
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = goal_handle.getGoal();

    // Goal does not command the right number of actuators
    if (goal->trajectory.joint_names.size() != m_joint_names.size())
    {
        ROS_ERROR("Goal commands %lu actuators, but arm has %lu.", goal->trajectory.joint_names.size(), m_joint_names.size());
        return false;
    }

    // Goal does not command the right actuators
    if (m_joint_names != goal->trajectory.joint_names)
    {
        ROS_ERROR("There is a mismatch between the goal's joint names and the action server's joint names.");
        ROS_INFO("Action server joint names are :");
        for (auto j : m_joint_names)
        {
            std::cout << j << ", ";
        }
        std::cout << std::endl;
        ROS_INFO("Goal joint names are :");
        for (auto j : goal->trajectory.joint_names)
        {
            std::cout << j << ", ";
        }
        std::cout << std::endl;
        return false;
    }
    
    return true;
}

bool JointTrajectoryActionServer::is_goal_tolerance_respected(bool enable_prints, bool check_time_tolerance)
{
    // Get feedback from arm (retry if timeouts occur)
    bool is_goal_respected = true;
    Kinova::Api::BaseCyclic::Feedback feedback;
    bool got_feedback = true;
    while (!got_feedback)
    {
        try
        {
            feedback = m_base_cyclic->RefreshFeedback();
            got_feedback = true;
        }
        catch(const std::exception& e)
        {
        }
    }
    auto goal = m_goal.getGoal();

    // Check the goal_time_tolerance for trajectory execution
    if (check_time_tolerance)
    {
        double actual_trajectory_duration = std::chrono::duration<double>(m_trajectory_end_time - m_trajectory_start_time).count();
        double desired_trajectory_duration = goal->trajectory.points.at(goal->trajectory.points.size()-1).time_from_start.toSec() + STARTING_POINT_ARBITRARY_DURATION;
        double time_tolerance = goal->goal_time_tolerance.toSec() == 0.0 ? m_default_goal_time_tolerance : goal->goal_time_tolerance.toSec();
        if (actual_trajectory_duration > desired_trajectory_duration + time_tolerance )
        {
            if (enable_prints)
                ROS_ERROR("Goal duration tolerance was exceeded. Maximum desired duration was %f seconds and actual duration was %f", desired_trajectory_duration + time_tolerance, actual_trajectory_duration);
            return false;
        }
        else if (actual_trajectory_duration < desired_trajectory_duration - time_tolerance)
        {
            if (enable_prints)
                ROS_ERROR("Goal duration threshold was not reached. Minimum desired duration was %f seconds and actual duration was %f", desired_trajectory_duration - time_tolerance, actual_trajectory_duration);
            return false;
        }
    }

    // If position tolerances were specified, use them.
    // If the goal->goal_tolerance vector is empty, fill it with default values
    std::vector<double> goal_tolerances;
    if (goal->goal_tolerance.empty())
    {
        ROS_DEBUG("Goal did not specify tolerances, using default tolerance of %f radians for every joint.", m_default_goal_tolerance);
        for (int i = 0; i < m_joint_names.size(); i++)
        {
            goal_tolerances.push_back(m_default_goal_tolerance);
        }
    }
    else
    {
        for (auto tol : goal->goal_tolerance)
        {
            goal_tolerances.push_back(tol.position);
        }
    }

    // Check the joint tolerances on the goal's end position
    int current_index = 0;
    for (auto act: feedback.actuators())
    {
        double actual_position = act.position(); // in degrees
        double desired_position = KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(goal->trajectory.points.at(goal->trajectory.points.size()-1).positions[current_index]));
        double tolerance = 0.0;

        if (goal_tolerances[current_index] == -1.0)
        {
            current_index++;
            continue; // no tolerance set for this joint
        }
        else
        {
            tolerance = KortexMathUtil::toDeg(goal_tolerances[current_index]);
        }

        double error = KortexMathUtil::wrapDegreesFromZeroTo360(std::min(fabs(actual_position - desired_position), fabs(fabs(actual_position - desired_position) - 360.0)));
        if (error > tolerance)
        {
            is_goal_respected = false;
            if (enable_prints)
                ROS_ERROR("The tolerance for joint %u was not met. Desired position is %f and actual position is %f", current_index + 1, desired_position, actual_position);
        }
        current_index++;
    }

    return is_goal_respected;
}

void JointTrajectoryActionServer::stop_all_movement()
{
    ROS_INFO("Calling Stop on the robot.");
    try
    {
        m_base->Stop();
    }
    catch(const Kinova::Api::KBasicException& e)
    {
        ROS_WARN("Stop failed : %s", e.what());
    }
}

void JointTrajectoryActionServer::set_server_state(ActionServerState s)
{
    std::lock_guard<std::mutex> guard(m_server_state_lock);
    ActionServerState old_state = m_server_state;
    m_server_state = s;
    // If we're going to IDLE, set back the soft limits
    if (s == ActionServerState::IDLE)
    {
        setAngularTrajectorySoftLimits(m_soft_limits);
    }
    ROS_INFO("State changed from %s to %s\n", actionServerStateNames[int(old_state)], actionServerStateNames[int(s)]);
}

AngularTrajectorySoftLimits JointTrajectoryActionServer::getAngularTrajectorySoftLimits()
{
    const Kinova::Api::ControlConfig::ControlMode target_control_mode = Kinova::Api::ControlConfig::ANGULAR_TRAJECTORY;

    Kinova::Api::ControlConfig::ControlModeInformation info;
    info.set_control_mode(target_control_mode);
    auto soft_limits = m_control_config->GetKinematicSoftLimits(info);

    // Set Joint Speed limits to max
    Kinova::Api::ControlConfig::JointSpeedSoftLimits vel;
    vel.set_control_mode(target_control_mode);
    for (auto j : soft_limits.joint_speed_limits())
    {
        vel.add_joint_speed_soft_limits(j);
    }

    // Set Joint Acceleration limits to max
    Kinova::Api::ControlConfig::JointAccelerationSoftLimits acc;
    acc.set_control_mode(target_control_mode);
    for (auto j : soft_limits.joint_acceleration_limits())
    {
        acc.add_joint_acceleration_soft_limits(j);
    }

    return AngularTrajectorySoftLimits(vel, acc);
}

void JointTrajectoryActionServer::setAngularTrajectorySoftLimitsToMax()
{
    ROS_DEBUG("Setting soft limits to hard");
    try
    {
        const Kinova::Api::ControlConfig::ControlMode target_control_mode = Kinova::Api::ControlConfig::ANGULAR_TRAJECTORY;
        
        // Set Joint Speed limits to max
        Kinova::Api::ControlConfig::JointSpeedSoftLimits jpsl;
        jpsl.set_control_mode(target_control_mode);
        for (auto j : m_hard_limits.joint_speed_limits())
        {
            jpsl.add_joint_speed_soft_limits(j);
        }
        m_control_config->SetJointSpeedSoftLimits(jpsl);

        // Set Joint Acceleration limits to max
        Kinova::Api::ControlConfig::JointAccelerationSoftLimits jasl;
        jasl.set_control_mode(target_control_mode);
        for (auto j : m_hard_limits.joint_acceleration_limits())
        {
            jasl.add_joint_acceleration_soft_limits(j);
        }
        m_control_config->SetJointAccelerationSoftLimits(jasl);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        ROS_WARN("Kortex exception while setting the angular soft limits");
        ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        ROS_WARN("Error description: %s\n", ex.what());
    }
}

void JointTrajectoryActionServer::setAngularTrajectorySoftLimits(const AngularTrajectorySoftLimits& limits)
{
    ROS_DEBUG("Setting back soft limits");
    if (!limits.empty())
    {
        try
        {
            // Set Joint Speed limits
            m_control_config->SetJointSpeedSoftLimits(limits.m_vel);

            // Set Joint Acceleration limits to max
            m_control_config->SetJointAccelerationSoftLimits(limits.m_acc);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            ROS_WARN("Kortex exception while setting the angular soft limits");
            ROS_WARN("Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
            ROS_WARN("Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
            ROS_WARN("Error description: %s\n", ex.what());
        }
    }
}
