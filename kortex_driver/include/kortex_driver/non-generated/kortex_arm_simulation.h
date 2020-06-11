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

#include "kortex_driver/non-generated/kortex_math_util.h"

class KortexArmSimulation
{
  public:
    KortexArmSimulation() = delete;
    KortexArmSimulation(ros::NodeHandle& nh);
    ~KortexArmSimulation();

    // Handlers for simulated Kortex API functions
    // TODO Add handlers

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

    // ROS and thread objects to publish the feedback from the robot
    KortexMathUtil m_math_util;

    // Helper functions
    bool IsGripperPresent() const {return !m_gripper_name.empty();}
};

#endif //_KORTEX_ARM_SIMULATION_H_
