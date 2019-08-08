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

# Kortex Driver

# **Note:** There have been many changes made between versions 1.1.7 and 2.0.0 of the ROS driver. You can view the changes and learn the steps to follow to adapt your code in [this section](#compatibility).

<!-- MarkdownTOC -->
## Table of contents

1. [Overview](#overview)
1. [Usage](#usage)
1. [Topics](#topics)
1. [Services](#services)
1. [Compatibility break between v1.1.X and v2.0.X](#compatibility)
1. [Generation (advanced)](#generation)

<!-- /MarkdownTOC -->

<a id="overview"></a>
## Overview
This node allows communication between a ROS node and a Kinova Gen3 Ultra lightweight robot.

### License

The source code is released under a [BSD 3-Clause license](../LICENSE).

**Author: Kinova inc.<br />
Affiliation: [Kinova inc.](https://www.kinovarobotics.com/)<br />
Maintainer: Kinova inc. support@kinovarobotics.com**

This package has been tested under ROS Kinetic and Ubuntu 16.04.

<a id="usage"></a>
## Usage

The `kortex_driver` node is the node responsible for the communication between the ROS network and the Kortex-compatible Kinova robots. It publishes topics that users can subscribe to. It also advertises services that users can call from the command line or from their own code to configure or control the robot arm or its sub-devices (actuators, vision module, interface module).  

**Arguments**:
- **arm** : Name of your robot arm model. See the `kortex_description/arms` folder to see the available robot models. The default value is **gen3**.
- **gripper** : Name of your robot arm's tool / gripper. See the `kortex_description/grippers` folder to see the available end effector models (or to add your own). The default value is **""** and the only other supported option is **robotiq_2f_85** for now.
- **robot_name** : This is the namespace in which the driver will run. It defaults to **my_$(arg arm)** (so "my_gen3" for arm="gen3").
- **ip_address** : The IP address of the robot you're connecting to. The default value is **192.168.1.10**.
- **cyclic_data_publish_rate** : Publish rate of the *base_feedback* and *joint_state* topics, in Hz. The default value is **100** Hz.
- **api_rpc_timeout_ms** : The default X-axis position of the robot in Gazebo. The default value is **0.0**.
- **api_session_inactivity_timeout_ms** : The duration after which the robot will clean the client session if the client hangs up the connection brutally (should not happen with the ROS driver). The default value is **35000** ms and is not normally changed.
- **api_connection_inactivity_timeout_ms** : The duration after which a connection is destroyed by the robot if no communication is detected between the client and the robot. The default value is **20000** ms and is not normally changed.
- **start_rviz** : If this argument is true, RViz will be launched. The default value is **true**.
- **start_moveit** : If this argument is true, a MoveIt! MoveGroup will be launched for the robot. The default value is **true**.

- **default_goal_time_tolerance** : The default goal time tolerance for the `FollowJointTrajectory` action server, in seconds. This value is used if no default goal time tolerance is specified in the trajectory. The default value is **0.5** seconds.
- **default_goal_tolerance** : The default goal tolerance for the `FollowJointTrajectory` action server, in degrees. This value is used if no default goal tolerance is specified in the trajectory for the joint positions reached at the end of the trajectory. The default value is **0.5** degrees.

To launch it with default arguments, run the following command in a terminal : 

`roslaunch kortex_driver kortex_driver.launch`

To launch it with optional arguments, specify the argument name, then ":=", then the value you want. For example, : 

`roslaunch kortex_driver kortex_driver.launch ip_address:=10.0.100.239 start_rviz:=false robot_name:=terminator`

You can also have a look at the [roslaunch documentation](http://wiki.ros.org/roslaunch/Commandline%20Tools) for more details.

If everything goes well, you will see a "**The Kortex driver has been initialized correctly!**" message. If you also start MoveIt!, the `kortex_driver` output may be flooded in the `move_group` output, so pay attention to the warning and error messages! If the node fails to start for any reason, you will get an error message followed by a "**process has died**" message.

<a id="topics"></a>
## Topics

### Robot feedback topics

The robot feedback topics are always published by the `kortex_driver`. You don't have to activate them. 

* **`/your_robot_name/kortex_error`**

    Every Kortex error will be published here. You can see the message description [here](msg/non_generated/KortexError.msg).

* **`/your_robot_name/base_feedback`**

    The feedback from the robot is published on this topic at a rate of **cyclic_data_publish_rate**. You can see the message description [here](msg/generated/base_cyclic/BaseCyclic_Feedback.msg).

* **`/your_robot_name/joint_state`**

    The feedback from the robot is converted to a [sensor_msgs/JointState](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html) and published on this topic at a rate of **cyclic_data_publish_rate**.

### Notification topics
    
The notification topics are only published by the `kortex_driver` if you activate them by first calling an activation service. Once activated, a notification topic will be activated until the node is shutdown. 

Subscribing to all the notifications causes a heavy load on the robot CPU. That is why the notification topics were designed in such a way. The users also typically only use one or two notifications, if at all. 

For example, if a user wants to subscribe to the **/my_robot_name/network_topic** (the message type is [NetworkNotification](msg/generated/base/NetworkNotification.msg)), he will have to: 
1. Call the **/my_robot_name/base/activate_publishing_of_network_notification** service to enable the publishing of the topic
2. Subscribe to the **/my_robot_name/network_topic** topic
3. Process the notifications when he receives them in his code

<a id="services"></a>
## Services
Most of the services supported by this node are generated from the [C++ Kortex API](https://github.com/Kinovarobotics/kortex). You can find the documentation [here](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/index.md).

### Understanding packages

The *.srv* files are generated in different sub-folders depending on the sub-module they affect. For example, all the RPC calls used to configure the vision module are generated in `srv/generated/vision_config` and all the RPC calls common to all devices are generated in `srv/generated/device_config`. Here is a list of the packages with a short explanation of the services they have to offer: 

* **actuator_config** : This package contains the functions used to configure a single actuator. 
**Note:** To choose the actuator you want to configure, you have to call the **/my_robot_name/actuator_config/set_device_id** service and specify the device identifier of the actuator you want to configure. You get the device identifiers of actuators when you launch the node, when you parse the output of the [ReadAllDevices](srv/generated/device_manager/ReadAllDevices.srv) service or in the Kinova Kortex *Web App*.

* **base** : This package contains :
    * Services to read and update the configuration of the robot
    * Services to send high level commands to the robot
    * Services to read and update the Product Configuration
    * Services to activate the publishing of notifications
    * Services to read and update the user-related information
**Note:** The base high level commands are treated every 25 ms inside the robot. High level control cannot be achieved at a rate faster than 40 Hz for now.

* **control_config** : This package contains the functions used to configure the control-related features on the robot. This includes : 
    * Reading and setting the cartesian reference frame
    * Reading and setting the gravity vector
    * Reading and setting the payload information
    * Reading and setting the tool configuration

* **device_config** : This package contains the functions used to configure a generic Kortex device. This includes : 
    * Reading and setting safety configurations
    * Reading general information on the specified device (software versions, serial numbers, MAC address, IPv4 settings, etc.)
**Note:** To choose the device you want to configure, you have to call the **/my_robot_name/device_config/set_device_id** service and specify the device identifier of the device you want to configure. You get the device identifiers when you launch the node, when you parse the output of the [ReadAllDevices](srv/generated/device_manager/ReadAllDevices.srv) service or in the WebApp.

* **device_manager** : This package contains [ReadAllDevices](srv/generated/device_manager/ReadAllDevices.srv) service, which is used to get the list of connected device and various informations on each device. 

* **interconnect_config** : This package contains the functions used to configure the interface module on the robot.
**Note:** You don't have to call the `SetDeviceID` service before calling the **interconnect_config** services, because the `kortex_driver` node goes through the list of connected devices and automatically sets the correct device ID for the **interconnect_config** services.

* **vision_config** : This package contains the functions used to configure the vision module on the robot.
**Note:** You don't have to call the SetDeviceID service before calling the **vision_config** services, because the kortex_driver node goes through the list of connected devices and automatically sets the correct device ID for the **vision_config** services.

<a id="compatibility"></a>
## Compatibility break between v1.1.7 and v2.0.0

Many things have been changed in the ros_kortex repository between versions 1.1.7 and 2.0.0 and you will have to modify your code if you don't want it to break.

* The `kortex_actuator_driver`, `kortex_vision_config_driver` and `kortex_device_manager` packages were removed and only the `kortex_driver` package remains (one driver to rule them all).
* Since we only have one driver and the ROS message generation does not deal with namespaces, the messages and services that are duplicated are now named differently. For example, the **Feedback** message exists within the `BaseCyclic`, `ActuatorCyclic`, `InterconnectCyclic` and `GripperCyclic` Protocol Buffers .proto files. In ROS, this is now translated as a "PackageName_" prefix before the message name. So, for the **Feedback** message, the **BaseCyclic_Feedback**, **ActuatorCyclic_Feedback**, **InterconnectCyclic_Feedback** and **GripperCyclic_Feedback** ROS messages have been automatically generated. You may encounter build errors (in C++) or runtime errors (in Python) because of this change. You can just go in the `msg/generated` folder and look for the problematic message to find its new name to change the occurrences in your code. 
* The services are now all **lowercase_with_underscores** instead of **UpperCase**.
* The services are now advertised in **/my_robot_name/my_package_name/desired_service** (see the [Services section](#services) to learn about the packages). You can also visualize it if you start the node and type `rosservice list` in a terminal.
* The topics are now all **lowercase_with_underscores** instead of **UpperCase**.
* The **/my_robot_name/base_feedback/joint_state** topic is now advertised as **/my_robot_name/joint_state**.
* The [kortex_driver launch file](launch/kortex_driver.launch) is now located in the `kortex_driver` package instead of the `kortex_bringup` package, which was deleted. Some arguments were added to the file.

<a id="generation"></a>
## Generation (**advanced**)

Some source code as well as most of the .MSG and .SRV files in this package are automatically generated, but the generated files are given on GitHub so that users don't have to generate them. However, if you have a special version of the Kortex API and want to generate those files yourself, it is possible. You will first need to follow the instructions to install Protocol Buffers.

The generation process is based on a custom `protoc` plugin. Basically, most of the generation process is in the [scripts/ros_kortex_generator.py](scripts/ros_kortex_generator.py). Before launching the generation ensure that you have the Python JINJA2 module installed.

To launch the generation of this package:

1. Open a terminal window.
2. Browse the /scripts directory of this package
3. Ensure that the generate_protobuf_wrapper_files.sh file can be executed. If not then run: <code>chmod +x generate_protobuf_wrapper_files.sh</code>
4. Run the command: <code>./generate_protobuf_wrapper_files.sh</code>
5. The result of the generation should be in the following folders:
    * `/include/kortex_driver/generated`
    * `/msg/generated`
    * `/src/generated`
    * `/srv/generated`

### Protos files
The **protos** directory contains the Protobuf files from which the MSG, SRV and source files are generated. The content of this folder should not be modified.

### Template files
The **templates** directory contains all the JINJA2 files needed by the `protoc` generator.
