#ifndef _KORTEX_ARM_SIMULATION_H_
#define _KORTEX_ARM_SIMULATION_H_

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

#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <unordered_map>
#include <thread>
#include <mutex>

#include "kortex_driver/non-generated/kortex_math_util.h"

#include "kortex_driver/ActionType.h"
#include "kortex_driver/KortexError.h"

#include "kortex_driver/CreateAction.h"
#include "kortex_driver/ReadAction.h"
#include "kortex_driver/ReadAllActions.h"
#include "kortex_driver/DeleteAction.h"
#include "kortex_driver/UpdateAction.h"
#include "kortex_driver/ExecuteActionFromReference.h"
#include "kortex_driver/ExecuteAction.h"
#include "kortex_driver/PauseAction.h"
#include "kortex_driver/StopAction.h"
#include "kortex_driver/ResumeAction.h"

#include "kortex_driver/PlayCartesianTrajectory.h"
#include "kortex_driver/Stop.h"
#include "kortex_driver/GetMeasuredCartesianPose.h"
#include "kortex_driver/SendTwistCommand.h"
#include "kortex_driver/PlayJointTrajectory.h"
#include "kortex_driver/SendJointSpeedsCommand.h"
#include "kortex_driver/SendGripperCommand.h"
#include "kortex_driver/ApplyEmergencyStop.h"

#include <moveit/move_group_interface/move_group_interface.h>

class KortexArmSimulation
{
  public:
    KortexArmSimulation() = delete;
    KortexArmSimulation(ros::NodeHandle& nh);
    ~KortexArmSimulation();
    std::unordered_map<uint32_t, kortex_driver::Action> GetActionsMap() const;
    int GetDOF() const {return m_degrees_of_freedom;}

    // Handlers for simulated Kortex API functions
    // Actions API
    kortex_driver::CreateAction::Response CreateAction(const kortex_driver::CreateAction::Request& req);
    kortex_driver::ReadAction::Response ReadAction(const kortex_driver::ReadAction::Request& req);
    kortex_driver::ReadAllActions::Response ReadAllActions(const kortex_driver::ReadAllActions::Request& req);
    kortex_driver::DeleteAction::Response DeleteAction(const kortex_driver::DeleteAction::Request& req);
    kortex_driver::UpdateAction::Response UpdateAction(const kortex_driver::UpdateAction::Request& req);
    kortex_driver::ExecuteActionFromReference::Response ExecuteActionFromReference(const kortex_driver::ExecuteActionFromReference::Request& req);
    kortex_driver::ExecuteAction::Response ExecuteAction(const kortex_driver::ExecuteAction::Request& req);
    kortex_driver::StopAction::Response StopAction(const kortex_driver::StopAction::Request& req);
    // Other RPCs
    kortex_driver::PlayCartesianTrajectory::Response PlayCartesianTrajectory(const kortex_driver::PlayCartesianTrajectory::Request& req);
    kortex_driver::SendTwistCommand::Response SendTwistCommand(const kortex_driver::SendTwistCommand::Request& req);
    kortex_driver::PlayJointTrajectory::Response PlayJointTrajectory(const kortex_driver::PlayJointTrajectory::Request& req);
    kortex_driver::SendJointSpeedsCommand::Response SendJointSpeedsCommand(const kortex_driver::SendJointSpeedsCommand::Request& req);
    kortex_driver::SendGripperCommand::Response SendGripperCommand(const kortex_driver::SendGripperCommand::Request& req);
    kortex_driver::Stop::Response Stop(const kortex_driver::Stop::Request& req);
    kortex_driver::ApplyEmergencyStop::Response ApplyEmergencyStop(const kortex_driver::ApplyEmergencyStop::Request& req);

  private:

    ros::NodeHandle m_node_handle;

    // Publishers
    ros::Publisher m_pub_action_topic;

    // Subscribers
    ros::Subscriber m_sub_joint_trajectory_controller_state;

    // Action clients
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> m_follow_joint_trajectory_action_client;

    // Namespacing and prefixing information
    std::string m_prefix;
    std::string m_robot_name;

    // Arm and gripper information
    std::string m_arm_name;
    std::vector<std::string> m_arm_joint_names;
    std::string m_gripper_name;
    std::vector<std::string> m_gripper_joint_names;
    std::vector<float> m_gripper_joint_limits_max;
    std::vector<float> m_gripper_joint_limits_min;
    int m_degrees_of_freedom;

    // Action-related
    std::unordered_map<uint32_t, kortex_driver::Action> m_map_actions;

    // Math utility
    KortexMathUtil m_math_util;

    // Threading
    std::atomic<bool> m_is_action_being_executed;
    std::atomic<bool> m_action_preempted;
    std::thread m_action_executor_thread;

    // MoveIt-related
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_moveit_arm_interface;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_moveit_gripper_interface;

    // Subscription callbacks and data structures with their mutexes
    void cb_joint_trajectory_controller_state(const control_msgs::JointTrajectoryControllerState& state);
    control_msgs::JointTrajectoryControllerState m_current_state;
    std::mutex m_state_mutex;

    // Helper functions
    bool IsGripperPresent() const {return !m_gripper_name.empty();}
    void CreateDefaultActions();

    // Executors
    void JoinThreadAndCancelAction();
    void PlayAction(const kortex_driver::Action& action);
    kortex_driver::KortexError ExecuteReachJointAngles(const kortex_driver::Action& action);
    kortex_driver::KortexError ExecuteReachPose(const kortex_driver::Action& action);
    kortex_driver::KortexError ExecuteSendJointSpeeds(const kortex_driver::Action& action);
    kortex_driver::KortexError ExecuteSendTwist(const kortex_driver::Action& action);
    kortex_driver::KortexError ExecuteSendGripperCommand(const kortex_driver::Action& action);
    kortex_driver::KortexError ExecuteTimeDelay(const kortex_driver::Action& action);
};

#endif //_KORTEX_ARM_SIMULATION_H_
