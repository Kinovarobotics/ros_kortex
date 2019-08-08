#ifndef _KORTEX_ROBOTIQ_GRIPPER_COMMAND_ACTION_SERVER_H_
#define _KORTEX_ROBOTIQ_GRIPPER_COMMAND_ACTION_SERVER_H_

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

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <chrono>
#include <mutex>
#include <thread>

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "Errors.pb.h"

// Duration timeout for a gripper trajectory (in seconds) 
#define GRIPPER_TRAJECTORY_TIME_LIMIT 2.0

// Robotiq Gripper relative position precision 
// Augmented to 2/255 instead of 1/255 because sometimes there are false negatives with 1/255
#define ROBOTIQ_GRIPPER_RELATIVE_ERROR 2.0/255.0

class RobotiqGripperCommandActionServer
{
    public:
        RobotiqGripperCommandActionServer() = delete;
        RobotiqGripperCommandActionServer(const std::string& server_name, const std::string& gripper_joint_name, double gripper_joint_limit, ros::NodeHandle& nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic);
        ~RobotiqGripperCommandActionServer();
   
    private:
        // Members
        ros::NodeHandle m_node_handle;
        actionlib::ActionServer<control_msgs::GripperCommandAction> m_server;

        Kinova::Api::Common::NotificationHandle m_sub_action_notif_handle;

        Kinova::Api::Base::GripperCommand           m_cancel_gripper_command;

        Kinova::Api::Base::BaseClient*              m_base;
        Kinova::Api::BaseCyclic::BaseCyclicClient*  m_base_cyclic;

        std::string m_server_name;

        control_msgs::GripperCommandFeedback     m_feedback;
        actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle m_goal;
        std::chrono::system_clock::time_point m_trajectory_start_time;
        
        bool m_is_trajectory_running;
        std::mutex m_is_trajectory_running_lock;
        std::thread m_gripper_position_polling_thread;

        // ROS Params
        std::string m_gripper_joint_name;
        double      m_gripper_joint_limit;

        // Action Server Callbacks
        void goal_received_callback(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle new_goal_handle);
        void preempt_received_callback(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle goal_handle);

        // Polling thread function
        void gripper_position_polling_thread();

        // Private methods
        bool is_goal_acceptable(actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle goal_handle);
        bool is_goal_tolerance_respected();
        void stop_all_movement();
        void join_polling_thread();
        double relative_position_from_absolute(double absolute_position);
        double absolute_position_from_relative(double relative_position);
};

#endif