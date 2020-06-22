/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2020 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "kortex_driver/non-generated/kortex_arm_simulation.h"
#include "kortex_driver/ErrorCodes.h"
#include "kortex_driver/SubErrorCodes.h"
#include "kortex_driver/ActionNotification.h"
#include "kortex_driver/ActionEvent.h"
#include "kortex_driver/JointTrajectoryConstraintType.h"

#include "trajectory_msgs/JointTrajectory.h"

#include <kdl_parser/kdl_parser.hpp>

#include <set>
#include <chrono>

namespace 
{
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string GRIPPER_PLANNING_GROUP = "gripper";
    static constexpr unsigned int FIRST_CREATED_ACTION_ID = 10000;
    static const std::set<unsigned int> DEFAULT_ACTIONS_IDENTIFIERS{1,2,3};
    static constexpr double JOINT_TRAJECTORY_TIMESTEP_SECONDS = 0.01;
}

KortexArmSimulation::KortexArmSimulation(ros::NodeHandle& node_handle): m_node_handle(node_handle),
                                                                        m_map_actions{},
                                                                        m_is_action_being_executed{false},
                                                                        m_action_preempted{false}
{
    // Namespacing and prefixing information
    ros::param::get("~robot_name", m_robot_name);
    ros::param::get("~prefix", m_prefix);

    // Arm and gripper information
    ros::param::get("~dof", m_degrees_of_freedom);
    ros::param::get("~arm", m_arm_name);
    ros::param::get("~joint_names", m_arm_joint_names);
    ros::param::get("~maximum_velocities", m_arm_velocity_max_limits);
    ros::param::get("~maximum_accelerations", m_arm_acceleration_max_limits);
    
    for (auto s : m_arm_joint_names)
    {
        s.insert(0, m_prefix);
    }
    ros::param::get("~gripper", m_gripper_name);
    if (IsGripperPresent())
    {
        ros::param::get("~gripper_joint_names", m_gripper_joint_names);
        for (auto s : m_gripper_joint_names)
        {
            s.insert(0, m_prefix);
        }
        ros::param::get("~gripper_joint_limits_max", m_gripper_joint_limits_max);
        ros::param::get("~gripper_joint_limits_min", m_gripper_joint_limits_min);
    }

    // Print out simulation configuration
    ROS_INFO("Simulating arm with following characteristics:");
    ROS_INFO("Arm type : %s", m_arm_name.c_str());
    ROS_INFO("Gripper type : %s", m_gripper_name.empty() ? "None" : m_gripper_name.c_str());
    ROS_INFO("Arm namespace : %s", m_robot_name.c_str());
    ROS_INFO("URDF prefix : %s", m_prefix.c_str());

    // Building the KDL chain from the robot description
    // The chain goes from 'base_link' to 'tool_frame'
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("robot_description", tree))
    {
        const std::string error_string("Failed to parse robot_description parameter to build the kinematic tree!"); 
        ROS_ERROR("%s", error_string.c_str());
        throw(std::runtime_error(error_string));
    }
    if (!tree.getChain(m_prefix + "base_link", m_prefix + "tool_frame", m_chain))
    {
        const std::string error_string("Failed to extract kinematic chain from parsed tree!"); 
        ROS_ERROR("%s", error_string.c_str());
        throw(std::runtime_error(error_string));
    }
    m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
    m_ik_pos_solver.reset(new KDL::ChainIkSolverPos_NR(m_chain, *m_fk_solver, *m_ik_vel_solver));

    // Build the velocity profile for each joint using the max velocities and max accelerations
    for (int i = 0; i < GetDOF(); i++)
    {
        m_velocity_trap_profiles.push_back(KDL::VelocityProfile_Trap(m_arm_velocity_max_limits[i], m_arm_acceleration_max_limits[i]));
    }

    // Start MoveIt client
    m_moveit_arm_interface.reset(new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP));
    if (IsGripperPresent())
    {
        m_moveit_gripper_interface.reset(new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP));   
    }

    // Create default actions
    CreateDefaultActions();

    // Create publishers and subscribers
    static const std::string joint_trajectory_controller_topic = "/" + m_robot_name + "/" + m_prefix + m_arm_name + "_joint_trajectory_controller";
    m_pub_action_topic = m_node_handle.advertise<kortex_driver::ActionNotification>("action_topic", 1000);
    m_sub_joint_trajectory_controller_state = m_node_handle.subscribe(joint_trajectory_controller_topic + "/state", 1, &KortexArmSimulation::cb_joint_trajectory_controller_state, this);

    // Create and connect action clients
    m_follow_joint_trajectory_action_client.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(joint_trajectory_controller_topic + "/follow_joint_trajectory", true));
    m_follow_joint_trajectory_action_client->waitForServer();
}

KortexArmSimulation::~KortexArmSimulation()
{
    JoinThreadAndCancelAction();
}

kortex_driver::CreateAction::Response KortexArmSimulation::CreateAction(const kortex_driver::CreateAction::Request& req)
{
    auto new_action = req.input;
    unsigned int identifier = FIRST_CREATED_ACTION_ID;
    bool identifier_taken = true;
    // Find unique identifier for new action
    while (identifier_taken)
    {
        identifier_taken = m_map_actions.count(identifier) == 1;
        if (identifier_taken)
        {
            ++identifier;
        }
    }
    // Add Action to map if type is supported
    switch (new_action.handle.action_type)
    {
        case kortex_driver::ActionType::REACH_JOINT_ANGLES:
        case kortex_driver::ActionType::REACH_POSE:
        case kortex_driver::ActionType::SEND_GRIPPER_COMMAND:
        case kortex_driver::ActionType::TIME_DELAY:
            new_action.handle.identifier = identifier;
            new_action.handle.permission = 7;
            m_map_actions.emplace(std::make_pair(identifier, new_action));
            break;
        default:
            ROS_ERROR("Unsupported action type %d : could not create simulated action.", new_action.handle.action_type);
            break;
    }
    // Return ActionHandle for added action
    kortex_driver::CreateAction::Response response;
    response.output = new_action.handle;
    return response;
}

kortex_driver::ReadAction::Response KortexArmSimulation::ReadAction(const kortex_driver::ReadAction::Request& req)
{
    auto input = req.input;
    kortex_driver::ReadAction::Response response;
    auto it = m_map_actions.find(input.identifier);
    if (it != m_map_actions.end())
    {
        response.output = it->second;
    }
    return response;
}

kortex_driver::ReadAllActions::Response KortexArmSimulation::ReadAllActions(const kortex_driver::ReadAllActions::Request& req)
{
    auto input = req.input;
    kortex_driver::ReadAllActions::Response response;
    kortex_driver::ActionList action_list;
    for (auto a : m_map_actions)
    {
        // If requested action type is specified and matches iterated action's type, add it to the list
        if (input.action_type == 0 || input.action_type == a.second.handle.action_type)
        {
            action_list.action_list.push_back(a.second);
        }
        
    }
    response.output = action_list;
    return response;
}

kortex_driver::DeleteAction::Response KortexArmSimulation::DeleteAction(const kortex_driver::DeleteAction::Request& req)
{
    auto handle = req.input;
    // If the action is not a default action
    if (DEFAULT_ACTIONS_IDENTIFIERS.find(handle.identifier) == DEFAULT_ACTIONS_IDENTIFIERS.end())
    {
        auto it = m_map_actions.find(handle.identifier);
        if (it != m_map_actions.end())
        {
            m_map_actions.erase(it);
            ROS_INFO("Simulated action #%u properly deleted.", handle.identifier);
        }
        else
        {
            ROS_WARN("Could not find simulated action #%u to delete in actions map.", handle.identifier);
        }
    }
    else
    {
        ROS_ERROR("Cannot delete default simulated actions.");
    }
    
    return kortex_driver::DeleteAction::Response();
}

kortex_driver::UpdateAction::Response KortexArmSimulation::UpdateAction(const kortex_driver::UpdateAction::Request& req)
{
    auto action = req.input;
    // If the action is not a default action
    if (DEFAULT_ACTIONS_IDENTIFIERS.find(action.handle.identifier) == DEFAULT_ACTIONS_IDENTIFIERS.end())
    {
        auto it = m_map_actions.find(action.handle.identifier);
        if (it != m_map_actions.end())
        {
            if (it->second.handle.action_type == action.handle.action_type)
            {
                it->second = action;
                ROS_INFO("Simulated action #%u properly updated.", action.handle.identifier);
            }
            else
            {
                ROS_ERROR("Cannot update action with different type.");
            }
        }
        else
        {
            ROS_ERROR("Could not find simulated action #%u to update in actions map.", action.handle.identifier);
        }
    }
    else
    {
       ROS_ERROR("Cannot update default simulated actions."); 
    }

    return kortex_driver::UpdateAction::Response();
}

kortex_driver::ExecuteActionFromReference::Response KortexArmSimulation::ExecuteActionFromReference(const kortex_driver::ExecuteActionFromReference::Request& req)
{
    auto handle = req.input;
    auto it = m_map_actions.find(handle.identifier);
    if (it != m_map_actions.end())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, it->second);
    }
    else
    {
        ROS_ERROR("Could not find action with given identifier %d", handle.identifier);
    }
    
    return kortex_driver::ExecuteActionFromReference::Response();
}

kortex_driver::ExecuteAction::Response KortexArmSimulation::ExecuteAction(const kortex_driver::ExecuteAction::Request& req)
{
    auto action = req.input;
    // Add Action to map if type is supported
    switch (action.handle.action_type)
    {
        case kortex_driver::ActionType::REACH_JOINT_ANGLES:
        case kortex_driver::ActionType::REACH_POSE:
        case kortex_driver::ActionType::SEND_GRIPPER_COMMAND:
        case kortex_driver::ActionType::TIME_DELAY:
            JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
            m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
            break;
        default:
            ROS_ERROR("Unsupported action type %d : could not execute simulated action.", action.handle.action_type);
            break;
    }

    return kortex_driver::ExecuteAction::Response();
}

kortex_driver::StopAction::Response KortexArmSimulation::StopAction(const kortex_driver::StopAction::Request& req)
{
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }

    m_follow_joint_trajectory_action_client->cancelAllGoals();
    
    return kortex_driver::StopAction::Response();
}

kortex_driver::PlayCartesianTrajectory::Response KortexArmSimulation::PlayCartesianTrajectory(const kortex_driver::PlayCartesianTrajectory::Request& req)
{
    auto constrained_pose = req.input;
    kortex_driver::Action action;
    action.name = "PlayCartesianTrajectory";
    action.handle.action_type = kortex_driver::ActionType::REACH_POSE;
    action.oneof_action_parameters.reach_pose.push_back(constrained_pose);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::PlayCartesianTrajectory::Response();
}

kortex_driver::SendTwistCommand::Response KortexArmSimulation::SendTwistCommand(const kortex_driver::SendTwistCommand::Request& req)
{
    auto twist_command = req.input;
    kortex_driver::Action action;
    action.name = "SendTwistCommand";
    action.handle.action_type = kortex_driver::ActionType::SEND_TWIST_COMMAND;
    action.oneof_action_parameters.send_twist_command.push_back(twist_command);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::SendTwistCommand::Response();
}

kortex_driver::PlayJointTrajectory::Response KortexArmSimulation::PlayJointTrajectory(const kortex_driver::PlayJointTrajectory::Request& req)
{
    auto constrained_joint_angles = req.input;
    kortex_driver::Action action;
    action.name = "PlayJointTrajectory";
    action.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    action.oneof_action_parameters.reach_joint_angles.push_back(constrained_joint_angles);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::PlayJointTrajectory::Response();
}

kortex_driver::SendJointSpeedsCommand::Response KortexArmSimulation::SendJointSpeedsCommand(const kortex_driver::SendJointSpeedsCommand::Request& req)
{
    auto joint_speeds = req.input;
    kortex_driver::Action action;
    action.name = "SendJointSpeedsCommand";
    action.handle.action_type = kortex_driver::ActionType::SEND_JOINT_SPEEDS;
    action.oneof_action_parameters.send_joint_speeds.push_back(joint_speeds);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::SendJointSpeedsCommand::Response();
}

kortex_driver::SendGripperCommand::Response KortexArmSimulation::SendGripperCommand(const kortex_driver::SendGripperCommand::Request& req)
{
    auto gripper_command = req.input;
    kortex_driver::Action action;
    action.name = "GripperCommand";
    action.handle.action_type = kortex_driver::ActionType::SEND_GRIPPER_COMMAND;
    action.oneof_action_parameters.send_gripper_command.push_back(gripper_command);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return kortex_driver::SendGripperCommand::Response();
}

kortex_driver::Stop::Response KortexArmSimulation::Stop(const kortex_driver::Stop::Request& req)
{
    // If an action is ongoing, cancel it first
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }
    m_follow_joint_trajectory_action_client->cancelAllGoals();
    return kortex_driver::Stop::Response();
}

kortex_driver::ApplyEmergencyStop::Response KortexArmSimulation::ApplyEmergencyStop(const kortex_driver::ApplyEmergencyStop::Request& req)
{
    // If an action is ongoing, cancel it first
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }
    m_follow_joint_trajectory_action_client->cancelAllGoals();
    return kortex_driver::ApplyEmergencyStop::Response();
}

void KortexArmSimulation::cb_joint_trajectory_controller_state(const control_msgs::JointTrajectoryControllerState& state)
{
    const std::lock_guard<std::mutex> lock(m_state_mutex);
    m_current_state = state;
}

void KortexArmSimulation::CreateDefaultActions()
{
    kortex_driver::Action retract, home, zero;
    kortex_driver::ConstrainedJointAngles retract_angles, home_angles, zero_angles;
    // Retract
    retract.handle.identifier = 1;
    retract.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    retract.handle.permission = 7;
    retract.name = "Retract";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("retract");
        double moveit_angle = named_target["joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        retract_angles.joint_angles.joint_angles.push_back(a);
    }
    retract.oneof_action_parameters.reach_joint_angles.push_back(retract_angles);
    // Home
    home.handle.identifier = 2;
    home.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    home.handle.permission = 7;
    home.name = "Home";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("home");
        double moveit_angle = named_target["joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        home_angles.joint_angles.joint_angles.push_back(a);
    }
    home.oneof_action_parameters.reach_joint_angles.push_back(home_angles);
    // Zero
    zero.handle.identifier = 3;
    zero.handle.action_type = kortex_driver::ActionType::REACH_JOINT_ANGLES;
    zero.handle.permission = 7;
    zero.name = "Zero";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("vertical");
        double moveit_angle = named_target["joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        zero_angles.joint_angles.joint_angles.push_back(a);
    }
    zero.oneof_action_parameters.reach_joint_angles.push_back(zero_angles);
    // Add actions
    m_map_actions.emplace(std::make_pair(retract.handle.identifier, retract));
    m_map_actions.emplace(std::make_pair(home.handle.identifier, home));
    m_map_actions.emplace(std::make_pair(zero.handle.identifier, zero));
}

kortex_driver::KortexError KortexArmSimulation::FillKortexError(uint32_t code, uint32_t subCode, const std::string& description) const
{
    kortex_driver::KortexError error;
    error.code = code;
    error.subCode = subCode;
    error.description = description;
    return error;
}

void KortexArmSimulation::JoinThreadAndCancelAction()
{
    m_action_preempted = true;
    if (m_action_executor_thread.joinable())
    {
        m_action_executor_thread.join();
    }
    m_action_preempted = false;
}

void KortexArmSimulation::PlayAction(const kortex_driver::Action& action)
{
    auto action_result = FillKortexError(kortex_driver::ErrorCodes::ERROR_NONE, kortex_driver::SubErrorCodes::SUB_ERROR_NONE);

    // Notify action started
    kortex_driver::ActionNotification start_notif;
    start_notif.handle = action.handle;
    start_notif.action_event = kortex_driver::ActionEvent::ACTION_START;
    m_pub_action_topic.publish(start_notif);
    m_is_action_being_executed = true;
    
    // Switch executor on the action type
    switch (action.handle.action_type)
    {
        case kortex_driver::ActionType::REACH_JOINT_ANGLES:
            action_result = ExecuteReachJointAngles(action);
            break;
        case kortex_driver::ActionType::REACH_POSE:
            action_result = ExecuteReachPose(action);
            break;
        case kortex_driver::ActionType::SEND_JOINT_SPEEDS:
            action_result = ExecuteSendJointSpeeds(action);
            break;
        case kortex_driver::ActionType::SEND_TWIST_COMMAND:
            action_result = ExecuteSendTwist(action);
            break;
        case kortex_driver::ActionType::SEND_GRIPPER_COMMAND:
            action_result = ExecuteSendGripperCommand(action);
            break;
        case kortex_driver::ActionType::TIME_DELAY:
            action_result = ExecuteTimeDelay(action);
            break;
        default:
            action_result = FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE, kortex_driver::SubErrorCodes::UNSUPPORTED_ACTION);
            break;
    }
    
    kortex_driver::ActionNotification end_notif;
    end_notif.handle = action.handle;
    // Action was cancelled by user
    if (m_action_preempted.load())
    {
        // Notify ACTION_ABORT
        end_notif.action_event = kortex_driver::ActionEvent::ACTION_ABORT;
        ROS_WARN("Action was aborted by user.");
    }
    // Action ended on its own
    else
    {
        if (action_result.code != kortex_driver::ErrorCodes::ERROR_NONE)
        {
            // Notify ACTION_ABORT
            end_notif.action_event = kortex_driver::ActionEvent::ACTION_ABORT;
            end_notif.abort_details = action_result.subCode;
            ROS_WARN("Action was failed : \nError code is %d\nSub-error code is %d\nError description is : %s", 
                        action_result.code,
                        action_result.subCode,
                        action_result.description.c_str());
        }
        else
        {
            // Notify ACTION_END
            end_notif.action_event = kortex_driver::ActionEvent::ACTION_END;
        }
    }
    m_pub_action_topic.publish(end_notif);
    
    m_is_action_being_executed = false;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteReachJointAngles(const kortex_driver::Action& action)
{
    auto result = FillKortexError(kortex_driver::ErrorCodes::ERROR_NONE, kortex_driver::SubErrorCodes::SUB_ERROR_NONE);
    if (action.oneof_action_parameters.reach_joint_angles.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                    kortex_driver::SubErrorCodes::INVALID_PARAM,
                                    "Error playing joint angles action : action is malformed.");
    }
    auto constrained_joint_angles = action.oneof_action_parameters.reach_joint_angles[0];
    if (constrained_joint_angles.joint_angles.joint_angles.size() != GetDOF())
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing joint angles action : action contains " + std::to_string(constrained_joint_angles.joint_angles.joint_angles.size()) + " joint angles but arm has " + std::to_string(GetDOF()));
    }

    // Initialize trajectory object
    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = m_prefix + "base_link";
    traj.header.stamp = ros::Time::now();
    for (int i = 0; i < constrained_joint_angles.joint_angles.joint_angles.size(); i++)
    {
        const std::string joint_name = m_prefix + "joint_" + std::to_string(i+1); //joint names are 1-based
        traj.joint_names.push_back(joint_name);
    }

    // Get current position
    trajectory_msgs::JointTrajectoryPoint current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state.actual;
    }

    // Transform kortex structure to trajectory_msgs to fill endpoint structure
    trajectory_msgs::JointTrajectoryPoint endpoint;
    for (int i = 0; i < constrained_joint_angles.joint_angles.joint_angles.size(); i++)
    {
        const double rad_wrapped_goal = m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(constrained_joint_angles.joint_angles.joint_angles[i].value));
        endpoint.positions.push_back(rad_wrapped_goal);
        endpoint.velocities.push_back(0.0);
        endpoint.accelerations.push_back(0.0);
    }

    // Calculate velocity profiles to know how much time this trajectory must last
    switch (constrained_joint_angles.constraint.type)
    {
        // If the duration is supplied, set the duration of the velocity profiles with that value
        case kortex_driver::JointTrajectoryConstraintType::JOINT_CONSTRAINT_DURATION:
        {
            // Error check on the given duration
            if (constrained_joint_angles.constraint.value <= 0.0f)
            {
                return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Invalid duration constraint : it has to be higher than 0.0!");
            }
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.positions[i], endpoint.positions[i], constrained_joint_angles.constraint.value);
            }
            endpoint.time_from_start = ros::Duration(constrained_joint_angles.constraint.value);
            ROS_DEBUG("Using supplied duration : %2.2f", constrained_joint_angles.constraint.value);
            break;
        }
        // If a max velocity is supplied for each joint, we need to find the limiting duration with this velocity constraint
        case kortex_driver::JointTrajectoryConstraintType::JOINT_CONSTRAINT_SPEED:
        {
            // Error check on the given velocity
            float max_velocity = m_math_util.toRad(constrained_joint_angles.constraint.value);
            if (max_velocity <= 0.0f)
            {
                return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Invalid velocity constraint : it has to be higher than 0.0!");
            }
            // Find the limiting duration with given velocity
            double max_duration = 0.0;
            for (int i = 0; i < GetDOF(); i++)
            {
                double velocity_ratio = std::min(1.0, double(max_velocity)/m_arm_velocity_max_limits[i]);
                m_velocity_trap_profiles[i].SetProfileVelocity(current.positions[i], endpoint.positions[i], velocity_ratio);
                max_duration = std::max(max_duration, m_velocity_trap_profiles[i].Duration());
                ROS_DEBUG("Joint %d moving from %2.2f to %2.2f gives duration %2.2f", i, current.positions[i], endpoint.positions[i], m_velocity_trap_profiles[i].Duration());
            }
            ROS_DEBUG("max_duration is : %2.2f", max_duration);
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.positions[i], endpoint.positions[i], max_duration);
            }
            endpoint.time_from_start = ros::Duration(max_duration);
            break;
        }
        default:
        {
            // Find the optimal duration based on actual velocity and acceleration limits
            double optimal_duration = 0.0;
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfile(current.positions[i], endpoint.positions[i]);
                optimal_duration = std::max(optimal_duration, m_velocity_trap_profiles[i].Duration());
                ROS_DEBUG("Joint %d moving from %2.2f to %2.2f gives duration %2.2f", i, current.positions[i], endpoint.positions[i], m_velocity_trap_profiles[i].Duration());
            }
            ROS_DEBUG("optimal_duration is : %2.2f", optimal_duration);
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.positions[i], endpoint.positions[i], optimal_duration);
            }
            endpoint.time_from_start = ros::Duration(optimal_duration);
            break;
        }
    }

    // Copy velocity profile data into trajectory using JOINT_TRAJECTORY_TIMESTEP_SECONDS timesteps
    // For each timestep
    for (double t = 0.0; t < m_velocity_trap_profiles[0].Duration(); t += JOINT_TRAJECTORY_TIMESTEP_SECONDS)
    {
        // Create trajectory point
        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = ros::Duration(t);
        // Add position, velocity, acceleration from each velocity profile
        for (int i = 0; i < GetDOF(); i++)
        {
            p.positions.push_back(m_velocity_trap_profiles[i].Pos(t));
            p.velocities.push_back(m_velocity_trap_profiles[i].Vel(t));
            p.accelerations.push_back(m_velocity_trap_profiles[i].Acc(t));
        }
        // Add trajectory point to goal
        traj.points.push_back(p);
    }
    // Finally, add endpoint to trajectory
    // Add position, velocity, acceleration from each velocity profile
    trajectory_msgs::JointTrajectoryPoint p;
    p.time_from_start = ros::Duration(m_velocity_trap_profiles[0].Duration());
    for (int i = 0; i < GetDOF(); i++)
    {
        p.positions.push_back(m_velocity_trap_profiles[i].Pos(m_velocity_trap_profiles[i].Duration()));
        p.velocities.push_back(m_velocity_trap_profiles[i].Vel(m_velocity_trap_profiles[i].Duration()));
        p.accelerations.push_back(m_velocity_trap_profiles[i].Acc(m_velocity_trap_profiles[i].Duration()));
    }
    // Add trajectory point to goal
    traj.points.push_back(p);

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }
    
    // Send goal
    control_msgs::FollowJointTrajectoryActionGoal goal;
    goal.goal.trajectory = traj;
    m_follow_joint_trajectory_action_client->sendGoal(goal.goal);

    // Wait for goal to be done, or for preempt to be called (check every 10ms)
    while(!m_action_preempted.load() && !m_follow_joint_trajectory_action_client->waitForResult(ros::Duration(0.01f))) {}

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_follow_joint_trajectory_action_client->cancelAllGoals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_follow_joint_trajectory_action_client->getResult();
        if (status->error_code != status->SUCCESSFUL)
        {
            result = FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::SubErrorCodes::INVALID_PARAM,
                                        status->error_string);
        }
    }
    return result;
}

// TODO Fill implementation
kortex_driver::KortexError KortexArmSimulation::ExecuteReachPose(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.reach_pose.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing pose action : action is malformed.");
    }
    auto constrained_pose = action.oneof_action_parameters.reach_pose[0];
    
    // TODO Handle constraints and warn if some cannot be applied in simulation
    // TODO Fill implementation to move simulated arm to Cartesian pose

    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteSendJointSpeeds(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_joint_speeds.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing joints speeds : action is malformed.");
    }
    auto joint_speeds = action.oneof_action_parameters.send_joint_speeds[0];
    if (joint_speeds.joint_speeds.size() != GetDOF())
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing joint speeds action : action contains " + std::to_string(joint_speeds.joint_speeds.size()) + " joint speeds but arm has " + std::to_string(GetDOF()));
    }

    // TODO Handle constraints and warn if some cannot be applied in simulation
    // TODO Fill implementation to move all actuators at given velocities

    return result;
}

// TODO Fill implementation
kortex_driver::KortexError KortexArmSimulation::ExecuteSendTwist(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_twist_command.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing twist action : action is malformed.");
    }
    auto twist = action.oneof_action_parameters.send_twist_command[0];

    // TODO Handle constraints and warn if some cannot be applied in simulation
    // TODO Fill implementation to move simulated arm at Cartesian twist
    
    return result;
}

// TODO Fill implementation
kortex_driver::KortexError KortexArmSimulation::ExecuteSendGripperCommand(const kortex_driver::Action& action)
{
    kortex_driver::KortexError result;
    result.code = kortex_driver::ErrorCodes::ERROR_NONE;
    result.subCode = kortex_driver::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_gripper_command.size() != 1)
    {
        return FillKortexError(kortex_driver::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::SubErrorCodes::INVALID_PARAM,
                                "Error playing gripper command action : action is malformed.");
    }
    auto gripper_command = action.oneof_action_parameters.send_gripper_command[0];

    // TODO Handle constraints and warn if some cannot be applied in simulation
    // TODO Handle velocity mode too?
    // TODO Fill implementation to move simulated gripper to given position

    return result;
}

kortex_driver::KortexError KortexArmSimulation::ExecuteTimeDelay(const kortex_driver::Action& action)
{
    auto result = FillKortexError(kortex_driver::ErrorCodes::ERROR_NONE,
                                kortex_driver::SubErrorCodes::SUB_ERROR_NONE);
    if (!action.oneof_action_parameters.delay.empty())
    {
        auto start = std::chrono::system_clock::now();
        uint32_t delay_seconds = action.oneof_action_parameters.delay[0].duration;
        // While not preempted and duration not elapsed
        while (!m_action_preempted.load() && (std::chrono::system_clock::now() - start) < std::chrono::seconds(delay_seconds))
        {
            // sleep a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else
    {
        result.code = kortex_driver::ErrorCodes::ERROR_DEVICE;
        result.subCode = kortex_driver::SubErrorCodes::INVALID_PARAM;
        result.description = "Error playing time delay action : action is malformed.";
    }
    return result;
}
