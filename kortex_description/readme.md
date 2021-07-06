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

# Kortex Description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots.

## Usage 

To load the description of a robot, you simply have to load the **ARM.xacro** or the **ARM_GRIPPER.xacro** file, with **ARM** being your arm's name (gen3, gen3_lite), and if you have a gripper, **GRIPPER** being your gripper's name (robotiq_2f_85, gen3_lite_2f).

**Arguments**:
- **sim** : If this argument is true, the Gazebo-specific files will be loaded. The default value is **false**.

For example:

- To load the Gen3 description with a Robotiq 2-F 85 gripper for simulation, you would put in your launch file : 
<code><param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_robotiq_2f_85.xacro sim:=true"\/></code>

- To load the Gen3 lite description, you would put in your launch file : 
<code><param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_lite_gen3_lite_2f.xacro sim:=false"\/></code>

## Tool frame

The `tool_frame` link refers to the tool frame used by the arm when it reports end effector position feedback.