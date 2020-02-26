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

#include "kortex_driver/non-generated/robotiq_gripper_command_action_server.h"

RobotiqGripperCommandActionServer::RobotiqGripperCommandActionServer(const std::string& server_name, const std::string& gripper_joint_name, double gripper_joint_limit_min, double gripper_joint_limit_max, ros::NodeHandle& nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic):
    m_server_name(server_name),
    m_gripper_joint_name(gripper_joint_name),
    m_gripper_joint_limit_min(gripper_joint_limit_min),
    m_gripper_joint_limit_max(gripper_joint_limit_max),
    m_node_handle(nh),
    m_server(nh, server_name, boost::bind(&RobotiqGripperCommandActionServer::goal_received_callback, this, _1), boost::bind(&RobotiqGripperCommandActionServer::preempt_received_callback, this, _1), false),
    m_base(base),
    m_base_cyclic(base_cyclic),
    m_is_trajectory_running(false)
{    
    // Construct cancel gripper command now so we can just send it whenever we need to
    m_cancel_gripper_command.set_mode(Kinova::Api::Base::GripperMode::GRIPPER_SPEED);
    auto g = m_cancel_gripper_command.mutable_gripper();
    auto f = g->add_finger();
    f->set_finger_identifier(0);
    f->set_value(0);

    // Ready to receive goal
    m_server.start();
}

RobotiqGripperCommandActionServer::~RobotiqGripperCommandActionServer()
{
    join_polling_thread();
}

void RobotiqGripperCommandActionServer::goal_received_callback(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle new_goal_handle)
{
    ROS_INFO("New goal received.");
    if (!is_goal_acceptable(new_goal_handle))
    {
        ROS_ERROR("Gripper Command Goal is rejected.");
        new_goal_handle.setRejected();
        return;
    }

    // Deal with active goals
    if (m_is_trajectory_running)
    {
        ROS_WARN("There is already an active goal. It is being cancelled.");
        stop_all_movement();
    }

    // Accept the goal
    m_goal = new_goal_handle;
    m_goal.setAccepted();

    // Construct the Protobuf gripper command
    control_msgs::GripperCommand ros_gripper_command = m_goal.getGoal()->command;
    Kinova::Api::Base::GripperCommand proto_gripper_command;

    proto_gripper_command.set_mode(Kinova::Api::Base::GripperMode::GRIPPER_POSITION);
    auto gripper = proto_gripper_command.mutable_gripper();
    // Position for a finger must be between relative (between 0 and 1), but position is absolute in the Goal coming from ROS
    double relative_position = m_math_util.relative_position_from_absolute(ros_gripper_command.position, m_gripper_joint_limit_min, m_gripper_joint_limit_max);
    auto finger = gripper->add_finger();
    finger->set_finger_identifier(0);
    finger->set_value(relative_position);
    
    // Send the gripper command to the robot
    m_base->SendGripperCommand(proto_gripper_command);
    m_is_trajectory_running_lock.lock();
    m_is_trajectory_running = true;
    m_is_trajectory_running_lock.unlock();

    // Start the thread to monitor the position
    join_polling_thread();
    m_gripper_position_polling_thread = std::thread(&RobotiqGripperCommandActionServer::gripper_position_polling_thread, this);
}

// Called in a separate thread when a preempt request comes in from the Action Client
void RobotiqGripperCommandActionServer::preempt_received_callback(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle goal_handle)
{
    ROS_WARN("Preempt received from client");
    if (m_is_trajectory_running)
    {
        stop_all_movement();
    }
    else 
    {
        ROS_WARN("Trajectory is not running but we received a pre-empt request from client.");
    }
}

// Called in a separate thread when a notification comes in
void RobotiqGripperCommandActionServer::gripper_position_polling_thread()
{
    std::chrono::system_clock::time_point now = m_trajectory_start_time = std::chrono::system_clock::now();
    Kinova::Api::BaseCyclic::Feedback feedback = m_base_cyclic->RefreshFeedback();
    //TODO When position feedback will be between 0 and 1, remove the *100 ot /100 in this file
    double previous_gripper_position = feedback.interconnect().gripper_feedback().motor(0).position() / 100.0; 
    double actual_gripper_position;
    int n_stuck = 0;

    // Break the loop if the trajectory is not running or if the trajectory lasted more than the timeout limit
    while(m_is_trajectory_running && (std::chrono::duration<double>(now - m_trajectory_start_time).count() < GRIPPER_TRAJECTORY_TIME_LIMIT))
    {
        // Every 50ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Get gripper position and find if we're stuck
        feedback = m_base_cyclic->RefreshFeedback();
        actual_gripper_position = feedback.interconnect().gripper_feedback().motor(0).position() / 100.0;
        if (fabs(actual_gripper_position - previous_gripper_position) > MAX_CONSECUTIVE_POSITION_DIFFERENCE)
        {
            n_stuck = 0;
        }
        else 
        {
            n_stuck++;
        }
        previous_gripper_position = actual_gripper_position;

        // If we're stuck, the trajectory is over
        if (n_stuck >= MAX_CONSECUTIVE_IDENTICAL_POSITIONS) // 200ms in the same position
        {
            m_is_trajectory_running_lock.lock();
            m_is_trajectory_running = false;
            m_is_trajectory_running_lock.unlock();
        }
    }

    // We got out this loop, meaning the trajectory is over or the time is up
    // The trajectory is not running, meaning it's finished or it was stopped
    if (!m_is_trajectory_running)
    {        
        if (is_goal_tolerance_respected())
        {
            m_goal.setSucceeded();
        }
        else
        {
            ROS_ERROR("we're finished");
            m_goal.setAborted();
        }
    }

    // Timeout was reached
    else
    {
        ROS_ERROR("BANG timeout");
        m_goal.setAborted();
    }

    return;
}

bool RobotiqGripperCommandActionServer::is_goal_acceptable(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle goal_handle)
{
    // First check if goal is valid
    if (!goal_handle.isValid())
    { 
        return false;
    }

    // Retrieve the goal
    control_msgs::GripperCommandGoalConstPtr goal = goal_handle.getGoal();

    // If the position is not in the joint limits range, reject the goal
    double relative_position = m_math_util.relative_position_from_absolute(goal_handle.getGoal()->command.position, m_gripper_joint_limit_min, m_gripper_joint_limit_max);
    if (relative_position > 1 || relative_position < 0)
    {
        return false;
    }

    return true;
}

bool RobotiqGripperCommandActionServer::is_goal_tolerance_respected()
{
    Kinova::Api::BaseCyclic::Feedback feedback = m_base_cyclic->RefreshFeedback();
    double actual_gripper_position = feedback.interconnect().gripper_feedback().motor(0).position() / 100.0;
    double goal_as_relative_position = m_math_util.relative_position_from_absolute(m_goal.getGoal()->command.position, m_gripper_joint_limit_min, m_gripper_joint_limit_max);

    ROS_INFO("%f and %f", actual_gripper_position, goal_as_relative_position);

    return fabs(actual_gripper_position - goal_as_relative_position) < MAX_GRIPPER_RELATIVE_ERROR;
}

void RobotiqGripperCommandActionServer::join_polling_thread()
{
    if (m_gripper_position_polling_thread.joinable())
    {
        m_gripper_position_polling_thread.join();
    }
}

void RobotiqGripperCommandActionServer::stop_all_movement()
{
    ROS_INFO("Sending zero-speed Gripper Command on the robot.");
    try
    {
        m_base->SendGripperCommand(m_cancel_gripper_command);
        m_is_trajectory_running_lock.lock();
        m_is_trajectory_running = false;
        m_is_trajectory_running_lock.unlock();
    }
    catch(const Kinova::Api::KBasicException& e)
    {
        ROS_ERROR("Sending a gripper speed of 0 failed : %s", e.what());
    }
}
