#ifndef _KORTEX_CARTESIAN_TRAJECTORY_ACTION_SERVER_H_
#define _KORTEX_CARTESIAN_TRAJECTORY_ACTION_SERVER_H_

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2021 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <chrono>
#include <mutex>

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "Errors.pb.h"
#include "kortex_driver/non-generated/kortex_math_util.h"
#include "kortex_driver/FollowCartesianTrajectoryAction.h"
#include "kortex_driver/FollowCartesianTrajectoryActionFeedback.h"
#include "kortex_driver/FollowCartesianTrajectoryActionGoal.h"
#include "kortex_driver/FollowCartesianTrajectoryActionResult.h"

class CartesianTrajectoryActionServer
{
    public:

        enum ActionServerState
        {
            INITIALIZING = 0,
            IDLE,
            PRE_PROCESSING_PENDING,
            PRE_PROCESSING_IN_PROGRESS,
            TRAJECTORY_EXECUTION_PENDING,
            TRAJECTORY_EXECUTION_IN_PROGRESS,
        };

        CartesianTrajectoryActionServer() = delete;
        CartesianTrajectoryActionServer(const std::string& server_name, ros::NodeHandle& nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic);
        ~CartesianTrajectoryActionServer();

        ActionServerState getState() {return m_server_state;};

        const char* actionServerStateNames[int(ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS) + 1] =
        {
            "INITIALIZING",
            "IDLE",
            "PRE_PROCESSING_PENDING",
            "PRE_PROCESSING_IN_PROGRESS",
            "TRAJECTORY_EXECUTION_PENDING",
            "TRAJECTORY_EXECUTION_IN_PROGRESS"
        };

    private:
        // Members
        ros::NodeHandle m_node_handle;
        actionlib::ActionServer<kortex_driver::FollowCartesianTrajectoryAction> m_server;

        Kinova::Api::Common::NotificationHandle m_sub_action_notif_handle;
        
        uint32_t m_currentActionID;

        Kinova::Api::Base::BaseClient* m_base;

        std::string m_server_name;

        actionlib::ActionServer<kortex_driver::FollowCartesianTrajectoryAction>::GoalHandle m_goal;
        std::chrono::system_clock::time_point m_trajectory_start_time;
        std::chrono::system_clock::time_point m_trajectory_end_time;

        std::mutex m_server_state_lock;
        std::mutex m_action_notification_thread_lock;
        ActionServerState m_server_state;

        // ROS Params
        std::string m_prefix;

        // Action Server Callbacks
        void goal_received_callback(actionlib::ActionServer<kortex_driver::FollowCartesianTrajectoryAction>::GoalHandle new_goal_handle);
        void preempt_received_callback(actionlib::ActionServer<kortex_driver::FollowCartesianTrajectoryAction>::GoalHandle goal_handle);

        // Kortex Notifications Callbacks
        void action_notif_callback(Kinova::Api::Base::ActionNotification notif);

        // Private methods
        bool is_goal_acceptable(actionlib::ActionServer<kortex_driver::FollowCartesianTrajectoryAction>::GoalHandle goal_handle);
        bool is_goal_tolerance_respected(bool enable_prints, bool check_time_tolerance);
        void stop_all_movement();

        void set_server_state(ActionServerState s);
};

#endif //_KORTEX_CARTESIAN_TRAJECTORY_ACTION_SERVER_H_