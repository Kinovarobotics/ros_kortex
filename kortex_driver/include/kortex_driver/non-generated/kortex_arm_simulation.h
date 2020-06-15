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

#include <unordered_map>

#include "kortex_driver/non-generated/kortex_math_util.h"

#include "kortex_driver/ActionType.h"

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

#include <moveit/move_group_interface/move_group_interface.h>

class KortexArmSimulation
{
  public:
    KortexArmSimulation() = delete;
    KortexArmSimulation(ros::NodeHandle& nh);
    ~KortexArmSimulation();
    std::unordered_map<uint32_t, kortex_driver::Action> GetActionsMap() const;

    // Handlers for simulated Kortex API functions
    // Actions API
    kortex_driver::CreateAction::Response CreateAction(const kortex_driver::CreateAction::Request& req);
    kortex_driver::ReadAction::Response ReadAction(const kortex_driver::ReadAction::Request& req);
    kortex_driver::ReadAllActions::Response ReadAllActions(const kortex_driver::ReadAllActions::Request& req);
    kortex_driver::DeleteAction::Response DeleteAction(const kortex_driver::DeleteAction::Request& req);
    kortex_driver::UpdateAction::Response UpdateAction(const kortex_driver::UpdateAction::Request& req);
    kortex_driver::ExecuteActionFromReference::Response ExecuteActionFromReference(const kortex_driver::ExecuteActionFromReference::Request& req);
    kortex_driver::ExecuteAction::Response ExecuteAction(const kortex_driver::ExecuteAction::Request& req);
    kortex_driver::PauseAction::Response PauseAction(const kortex_driver::PauseAction::Request& req);
    kortex_driver::StopAction::Response StopAction(const kortex_driver::StopAction::Request& req);
    // Sequences API
    // Velocity control RPCs

  private:

    ros::NodeHandle m_node_handle;

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

    // ROS and thread objects to publish the feedback from the robot
    KortexMathUtil m_math_util;

    // MoveIt-related
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_moveit_arm_interface;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_moveit_gripper_interface;

    // Helper functions
    bool IsGripperPresent() const {return !m_gripper_name.empty();}
    void CreateDefaultActions();
};

#endif //_KORTEX_ARM_SIMULATION_H_
