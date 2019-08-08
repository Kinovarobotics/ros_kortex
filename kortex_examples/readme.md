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

# Kortex Examples

<!-- MarkdownTOC -->

1. [Before running an example](#first_of_all)
2. [Actuator configuration examples](#actuator_config)
3. [Full arm movement examples](#full_arm)
4. [Vision module configuration examples](#vision_config)
5. [MoveIt! examples](#move_it)

<!-- /MarkdownTOC -->

<a id="first_of_all"></a>
## Before running an example

Before you run any example, make sure :
- You have already built the packages using `catkin_make`.
- You are physically connected to an arm (or you are connected over Wi-Fi).
- You have started the `kortex_driver` node by following the [instructions](../kortex_driver/readme.md). 
- The node started correctly and without errors.


<a id="actuator_config"></a>
## Actuator configuration examples
*Examples to show how to use actuator_config ROS services to configure a given actuator.*

The examples look for advertised services in the **my_gen3** namespace by default and configures the first actuator.

To run the C++ example: `rosrun kortex_examples example_actuator_configuration_cpp`

To run the Python example: `rosrun kortex_examples example_actuator_configuration.py`

If you started the `kortex_driver` node in a non-default namespace (not **my_gen3**) or if you want to test the example on another actuator than the first one, you will have to supply node parameters in the command line (the syntax doesn't change if you run the C++ or Python example) : 

`rosrun kortex_examples example_actuator_configuration_cpp _robot_name:=<your_own_namespace> _device_id:=<your_device_id>`

<a id="full_arm"></a>
## Full arm examples
*Examples to show how to use the base ROS services to move and configure the arm.*

The examples look for advertised services in the **my_gen3** namespace by default.

To run the C++ example: `rosrun kortex_examples example_full_arm_movement_cpp`

To run the Python example: `rosrun kortex_examples example_full_arm_movement.py`

If you started the `kortex_driver` node in a non-default namespace (not **my_gen3**), you will have to supply the node a parameter in the command line (the syntax doesn't change if you run the C++ or Python example) : 

`rosrun kortex_examples example_full_arm_movement_cpp _robot_name:=<your_own_namespace>`

<a id="vision_config"></a>
## Vision module configuration examples
*Examples to show how to use the vision_config ROS services to configure the vision module.*

The examples look for advertised services in the **my_gen3** namespace by default.

To run the C++ example: `rosrun kortex_examples example_vision_configuration_cpp`

To run the Python example: `rosrun kortex_examples example_vision_configuration.py`

If you started the `kortex_driver` node in a non-default namespace (not **my_gen3**), you will have to supply the node a parameter in the command line (the syntax doesn't change if you run the C++ or Python example) : 

`rosrun kortex_examples example_full_arm_movement_cpp _robot_name:=<your_own_namespace>`

<a id="move_it"></a>
## MoveIt! example
*Example to show how to use the Python MoveIt! API to move the arm.*

The example looks for advertised services and topics in the **my_gen3** namespace by default.

To run the example: `rosrun kortex_examples example_move_it_trajectories.py`

If you started the `kortex_driver` node in a non-default namespace (not **my_gen3**), you will have to supply the node your own namespace in the command line : 

`rosrun kortex_examples example_full_arm_movement_cpp __ns:=<your_own_namespace>`
