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

# Kortex Gazebo

## Overview
This package contains files to simulate the Kinova Gen3 and Gen3 lite robots in Gazebo.

### License

The source code is released under a [BSD 3-Clause license](../LICENSE).

**Author: Kinova inc.<br />
Affiliation: [Kinova inc.](https://www.kinovarobotics.com/)<br />
Maintainer: Kinova inc. support@kinovarobotics.com**

This package has been tested under ROS Noetic with Gazebo 11.

For older versions, checkout on corresponding branch : 

- [melodic-devel](https://github.com/Kinovarobotics/ros_kortex/tree/melodic-devel) for ROS Melodic and Gazebo 9. 
- [kinetic-devel](https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel) for ROS Kinetic and Gazebo 7 (the melodic-devel branch might work for this configuration).

## A word on stability of the models
 
The arms's controllers are simulated with the `gazebo_ros_control` package, which provides PID effort controllers for every joint. It is not an exact simulation of the real arms's behaviour. The simulated arms are stable in most of their workspace, although we have seen the Gen3 be a bit less stable in a very low inertia position (vertical). The gains are set in .yaml files in the `kortex_control` package, so they can be modified by end users.

Other disclaimers :
 - The simulation has not been tested with simulated payload.
 - No characterization was done to compare and quantify the performance of the simulated controllers and the real arm's controllers. The simulation only offers a **visually comparable experience** to the real arms. 
 - Very rarely, the base (fixed to the world frame) seems to lose its fixation and the arm goes unstable. We are currently looking for the root cause of this behavior, but we found that clicking **Edit ---> Reset Model Poses** (Shift + Ctrl + R) in Gazebo is a workaround.

## Usage

The [spawn_kortex_robot.launch file](launch/spawn_kortex_robot.launch) launches the arm simulation in [Gazebo](http://gazebosim.org), with [ros_control](http://wiki.ros.org/ros_control) controllers and [MoveIt!](https://moveit.ros.org/).
The launch can be parametrized with arguments : 

**Arguments**:
- **start_gazebo** : If this argument is false, Gazebo will not be launched within this launched file. It is useful if you already launched Gazebo yourself and just want to spawn the robot. The default value is **true** (Gazebo will be started).
- **gazebo_gui** : If this argument is false, only the Gazebo Server will be launched. The default value is **true**.
- **start_rviz** : If this argument is true, RViz will be launched. The default value is **true**.
- **x0** : The default X-axis position of the robot in Gazebo. The default value is **0.0**.
- **y0** : The default Y-axis position of the robot in Gazebo. The default value is **0.0**.
- **z0** : The default Z-axis position of the robot in Gazebo. The default value is **0.0**.
- **arm** : Name of your robot arm model. See the `kortex_description/arms` folder to see the available robot models. The default value is **gen3**.
- **gripper** : Name of your robot arm's tool / gripper. See the `kortex_description/grippers` folder to see the available end effector models (or to add your own). The default value is **""**. For Gen3, you can also put **robotiq_2f_85** or **robotiq_2f_140**. For Gen3 lite, you need to put **gen3_lite_2f**.
- **robot_name** : This is the namespace of the arm that is going to be spawned. It defaults to **my_$(arg arm)** (so my_gen3 for arm="gen3").
- **use_trajectory_controller** : If this argument is false, one `JointPositionController` per joint will be launched and the arm will offer a basic ROS Control interface to control every joint individually with topics. If this argument is true, a MoveIt! node will be started for the arm and the arm will offer a `FollowJointTrajectory` interface to control the arm (via a `JointTrajectoryController`). The default value is **true**.
- **use_sim_time** : If this value is true, Gazebo will use simulated time instead of system clock. The default value is **true**.
- **debug** : If this value is true, Gazebo will be launched in debug mode. This option is useful for debugging Gazebo-related issues that won't show in the terminal. The default value is **false**.
- **paused** : If this value is true, Gazebo will be started paused. The default value is **$(arg use_trajectory_controller)** because, when MoveIt! is enabled, Gazebo needs to be started paused to let the controllers initialize.

To launch it with default arguments, run the following command in a terminal : 

`roslaunch kortex_gazebo spawn_kortex_robot.launch`

To launch it with optional arguments, specify the argument name, then ":=", then the value you want. For example, : 

`roslaunch kortex_gazebo spawn_kortex_robot.launch start_rviz:=false x0:=1.0 y0:=2.55 use_trajectory_controller:=false`

You can also have a look at the [roslaunch documentation](http://wiki.ros.org/roslaunch/Commandline%20Tools) for more details.

## Gazebo

Gazebo loads an empty world an first, then the `spawn_model` node from the [gazebo_ros package](http://wiki.ros.org/gazebo_ros) is used to load the `robot_description` (taken from the Parameter Server) and spawn the robot in the simulator. 

## Controllers

The simulated arm is controlled with a `effort_controllers/JointTrajectoryController` from [ros_controllers](http://wiki.ros.org/ros_controllers).
This controller offers a [FollowJointTrajectory](http://wiki.ros.org/joint_trajectory_controller) interface to control the simulated arm, which is configured with [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html) so the trajectories outputed by the `move_group` node are sent to the robot in Gazebo and executed.
The RViz Motion Planning plugin offers simple motion planning support for the simulated robot. See the [kortex_move_it_config documentation](../kortex_move_it_config/readme.md) for more details.

There are also individual JointPositionController's for every joint, stopped by default. These are used by the kortex_arm_driver node to do velocity control. 

See the [kortex_driver package documentation](../kortex_driver/readme.md) for more details on how to use the simulated arm. 

## Initialization script

The package also uses a [Python script](./scripts/home_robot.py) to home the robot after the robot has been spawned. 
Gazebo is **launched with paused physics**. Otherwise, the arm would fall to the ground because the controllers are not fully loaded when the robot is spawned in the simulator. 
When everything is well loaded, the script unpauses Gazebo's physics and executes the robot's Home action.

## Plugins

To properly simulate the grippers in Gazebo, we use two Gazebo Plugins.

### Mimic joint plugin

Gazebo doesn't offer support for the URDF **mimic** tag. 
By loading one instance of [this plugin](../third_party/roboticsgroup_gazebo_plugins/README.md) per mimic joint, we are able to simulate those joints properly. The plugin parameters are specified with the [transmission elements for the Robotiq gripper](../kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_transmission_macro.xacro) and the [transmission elements for the Gen3 lite gripper](../kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_transmission_macro.xacro).

### Gazebo grasp plugin

Gazebo doesn't support grasping very well. By loading [this plugin](../third_party/gazebo-pkgs/README.md), we make sure objects grasped by the gripper will not fall. The plugin parameters are specified in with the [transmission elements for the Robotiq gripper](../kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_transmission_macro.xacro) and the [transmission elements for the Gen3 lite gripper](../kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_transmission_macro.xacro).
