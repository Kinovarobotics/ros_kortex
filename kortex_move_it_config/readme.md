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

# Kortex MoveIt! Config

## Overview
This folder contains all the auto-generated MoveIt! configuration ROS packages. These packages have been generated using the [MoveIt! Setup Assistant](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html).

## Naming

The packages that don't use a gripper are named `ARM_move_it_config`, where "ARM" is the name of the arm you are using. 
See the `kortex_description/arms` folder for a list of supported Kinova Kortex robots.

The packages that use a gripper are named `ARM_GRIPPER_move_it_config`, where "ARM" is the name of the arm you are using and "GRIPPER" is the name of the gripper you are using.  
See the `kortex_description/grippers` folder for a list of supported Kinova Kortex grippers. 

## Using MoveIt! with Kinova Kortex Robots

Upon launching the main launch file of a `move_it_config` package, `move_group.launch` (normally launched from the real arm driver's launch file and the simulation launch file), the [C++](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html) and [Python](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) interfaces for MoveIt will be enabled. 
You will be able to use motion planning, configure the planning scene, send trajectories and send pose goals to the simulated robot from your own ROS nodes.

You can take a look at our [MoveIt! Python example](../kortex_examples/src/move_it/example_move_it_trajectories.py) for a concrete example.  
