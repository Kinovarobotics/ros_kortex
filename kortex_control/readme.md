<!-- 
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed 
* under the terms of the BSD 3-Clause license. 
*
* Refer to the LICENSE file for details.
*
* -->

# Kortex Control

## Overview
This package contains the configuration files for the [ros_control controllers](http://wiki.ros.org/ros_control) used to control the simulated arms.  

## Loading and starting the controllers for simulation

The `joint_position_controllers.yaml` file for the chosen arm is loaded to the Parameter Server from the [spawn_kortex_robot.launch](../kortex_gazebo/launch/spawn_kortex_robot.launch) file. The `controller_manager` node is then called to load and start the controllers as [Gazebo plugins](http://wiki.ros.org/gazebo_ros_control). 

## ROS Control support for the real arm

ROS Control support for the Kinova Gen3 and Gen3 lite robots is not currently available and will be part of a future release.  
