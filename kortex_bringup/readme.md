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

# Kortex bringup

## Overview
This package contains the parametrized files that launch the nodes and load the correct parameters and robot description to the Parameter Server.

<!-- MarkdownTOC -->

1. [kortex_actuator_driver](#actuator-driver)
1. [kortex_device_manager](#device_manager)
1. [kortex_driver](#driver)
1. [kortex_vision_config_driver](#vision_config)

<!-- /MarkdownTOC -->


<a id="actuator-driver"></a>
## kortex_actuator_driver 

This file launches the kortex_actuator_driver node and, optionally, the kortex_device_manager node.
The launch can be parametrized with arguments : 

**Required argument**:
- **ip_address** : IP address of your device.

**Optional arguments**:
- **robot_name** : Namespace for your robot. The default value is **my_kortex_actuator**.
- **cycle_rate** : Kortex API rate (in Hz) for communicating with the device and publishing topics. The default value is **60**.
- **device_id** : Device ID of the actuator you want to control. If this argument is a non-empty string, the device routing mechanism will be activated. See [the kortex_actuator_driver node documentation](../kortex_actuator_driver/readme.md) for more details. The default value is an empty string (not activated). 
- **start_device_manager** : If this argument is true, the [kortex_device_manager node](../kortex_device_manager/readme.md) will also be started. The default value is **true**.


To launch it, run the following command in a terminal (but change the IP address for your own!) : 

<code>roslaunch kortex_bringup kortex_actuator_driver.launch ip_address:=192.168.1.10</code>

To launch it with optional arguments, run the following command in a terminal : 

<code>roslaunch kortex_bringup kortex_actuator_driver.launch ip_address:=192.168.1.10 robot_name:=my_extraordinary_robot cycle_rate:=50 device_id:=2</code>

<a id="device_manager"></a>
## kortex_device_manager

This file launches the kortex_device_manager node.
The launch can be parametrized with arguments : 

**Required argument**:
- **ip_address** : IP address of your device.

To launch it, run the following command in a terminal (but change the IP address for your own!) : 

<code>roslaunch kortex_bringup kortex_device_manager.launch ip_address:=192.168.1.10</code>

<a id="driver"></a>
## kortex_driver 

This file launches the kortex_actuator_driver node and, optionally, the kortex_device_manager node.
The launch can be parametrized with arguments : 

**Required argument**:
- **ip_address** : IP address of your device.

**Optional arguments**:
- **arm** : Name of your robot arm model. See the kortex_description/arms folder to learn about the available robot models. The default value is **gen3**.
- **gripper** : Name of your robot arm's end effector. See the kortex_description/grippers folder to learn about the available end effector models. The default value is an empty string for now because grippers aren't supported.
- **robot_name** : Namespace for your robot. The default value is **my_(arm argument)**.
- **cycle_rate** : Kortex API rate (in Hz) for communicating with the device and publishing topics. The default value is **60**.
- **start_device_manager** : If this argument is true, the [kortex_device_manager node](../kortex_device_manager/readme.md) will also be started. The default value is **true**.
- **start_rviz** : If this argument is true, a [RViz](http://wiki.ros.org/rviz) window will also be started. The default value is **true**.

To launch it, run the following command in a terminal (but change the IP address for your own!) : 

<code>roslaunch kortex_bringup kortex_driver.launch ip_address:=192.168.1.10</code>

To launch it with optional arguments, run the following command in a terminal : 

<code>roslaunch kortex_bringup kortex_driver.launch ip_address:=192.168.1.10 robot_name:=my_extraordinary_gen3_robot cycle_rate:=50 start_rviz:=false</code>

<a id="vision_config"></a>
## kortex_vision_config

This file launches the kortex_vision_config node and, optionally, the kortex_device_manager node.
The launch can be parametrized with arguments : 

**Required argument**:
- **ip_address** : IP address of your device.

**Optional arguments**:
- **robot_name** : Namespace for your robot. The default value is **my_vision_device**.
- **cycle_rate** : Kortex API rate (in Hz) for communicating with the device and publishing topics. The default value is **60**.
- **device_id** : Device ID of the vision device you want to communicate with. If this argument is a non-empty string, the device routing mechanism will be activated. See [the kortex_vision_config_driver node documentation](../kortex_vision_config_driver/readme.md) for more details. The default value is an empty string (not activated). 
- **start_device_manager** : If this argument is true, the [the kortex_device_manager node](../kortex_device_manager/readme.md) will also be started. The default value is **true**.

To launch it, run the following command in a terminal (but change the IP address for your own!) : 

<code>roslaunch kortex_bringup kortex_vision_config.launch ip_address:=192.168.1.10</code>

To launch it with optional arguments, run the following command in a terminal : 

<code>roslaunch kortex_bringup kortex_vision_config.launch ip_address:=192.168.1.10 robot_name:=my_extraordinary_vision_device cycle_rate:=50 start_device_manager:=false</code>

