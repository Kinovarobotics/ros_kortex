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

KortexArmSimulation::KortexArmSimulation(ros::NodeHandle& node_handle): m_node_handle(node_handle)
{
    // Namespacing and prefixing information
    ros::param::get("~robot_name", m_robot_name);
    ros::param::get("~prefix", m_prefix);

    // Arm and gripper information
    ros::param::get("~dof", m_degrees_of_freedom);
    ros::param::get("~arm", m_arm_name);
    ros::param::get("~joint_names", m_arm_joint_names);
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
}

KortexArmSimulation::~KortexArmSimulation()
{
}
