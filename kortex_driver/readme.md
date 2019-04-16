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

## Overview
This node allows communication between a node and a Gen3 robot. Use this package if you want to:

* Change basic configuration of the robot.
* Move the robot in the Cartesian space.
* Move the robot in the joint space.
* Activate the admittance mode.
* Move the robot using the **LOW\_LEVEL**\ (1 kHz\) control mode.
* Move the robot using the **LOW\_LEVEL\_BYPASS**\ mode.
* Access the cyclic data sporadically.

### License

The source code is released under a [BSD 3-Clause license](../LICENSE).

**Author: Kinova inc.<br />
Affiliation: [Kinova inc.](https://www.kinovarobotics.com/)<br />
Maintainer: Kinova inc. support@kinovarobotics.com**

This package has been tested under ROS Kinetic and Ubuntu 16.04.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Protobuf](https://developers.google.com/protocol-buffers/)

```cpp
git clone https://github.com/protocolbuffers/protobuf --branch 3.5.1.1   (you must use this specific version)
```
Follow these [instructions](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md) to build and install protobuf and its compiler.

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/Kinovarobotics/ros_kortex.git
        cd ../
        sudo ./src/ros_kortex/build.sh

## Usage

### Launch file
The launch file for this node can be found in the [kortex_bringup](../kortex_bringup/readme.md) package.


### Starting with rosrun

<code>rosrun kortex\_driver kortex\_driver 192.168.1.10 100</code>

In the command above, you would be running the kortex_driver node on an Gen3 robot with IP address 192.168.1.10. The cyclic data would be refreshed at 100 Hz.



## Nodes

### Published Topics

* **`/KortexError`**
    <p>Every Kortex error will be published here. </p>
    
    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | code | Error code, see enum in the ErrorCodes class. |
    | uint32 | subcode | Sub error code, see enum in the ErrorCodes class. |
    | string | description | Error details |

* **`/ConfigurationChangeTopic`**
    <p>Notification received when a configuration item change.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | event | Event type, see [ConfigurationNotificationEvent.msg](https://github.com/Kinovarobotics/ros_kortex/blob/master/kortex_driver/msg/ConfigurationNotificationEvent.msg). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the configuration change event. |
    | Connection | connection | Connection that caused the configuration change event. |
    
* **`/MappingInfoTopic`**
    <p>Notification received when a controller changes its active map.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | controller_identifier | Identifier of the controller, see [ConfigurationNotificationEvent.msg](https://github.com/Kinovarobotics/ros_kortex/blob/master/kortex_driver/msg/ControllerType.msg). |
    | MapHandle | active_map_handle | A handle to the new active map. |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the mapping event. |
    | Connection | connection | Connection that caused the mapping event. |

* **`/ControlModeTopic`**
    <p>Notification received when the control mode has been changed.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | control_mode | New control mode, see [ControlMode.msg](https://github.com/Kinovarobotics/ros_kortex/blob/master/kortex_driver/msg/ControlMode.msg). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the control mode event. |
    | Connection | connection | Connection that caused the control mode event. |

* **`/OperatingModeTopic`**
    <p>Notification received when the operating mode is changed.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | operating_mode | New operating mode, see [OperatingMode.msg](https://github.com/Kinovarobotics/ros_kortex/blob/master/kortex_driver/msg/OperatingMode.msg). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the operating mode event. |
    | Connection | connection | Connection that caused the operating mode event. |
    | DeviceHandle | device_handle | Device matching operating mode (if applicable). |

* **`/SequenceInfoTopic`**
    <p>Notification received when an event occured during a sequence.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | event_identifier | New operating mode, see [EventIdSequenceInfoNotification.msg](https://github.com/Kinovarobotics/ros_kortex/blob/master/kortex_driver/msg/ControllerType.msg). |
    | SequenceHandle | sequence_handle | Handle of the sequence that this event refers to. |
    | uint32 | task_index | Task index. |
    | uint32 | group_identifier | Specifies the order in which this task must be executed. |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the sequence event. |
    | uint32 | abort_details | Details if event_identifier is equal to ABORT. |
    | Connection | connection | Connection that caused the sequence event. |

* **`/ProtectionZoneTopic`**
    <p>Notification received when a protection zone event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | operating_mode | New operating mode, see [ProtectionZoneEvent.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Base_ProtectionZoneEvent.md#). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the protection zone event. |
    | Connection | connection | Connection that caused the protection zone event. |

* **`/ControllerTopic`**
    <p>Notification received when a controller event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the controller event. |
    | Connection | connection | Connection that caused the controller event. |

* **`/ActionTopic`**
    <p>Notification received when an action event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | action_event | New operating mode, see [ActionEvent.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Base_ActionEvent.md#). |
    | ActionHandle | handle | Identifies the action for which this event occured. |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the action event. |
    | uint32 | abort_details | Details if event_identifier is equal to ABORT. |
    | Connection | connection | Connection that caused the action event. |

* **`/RobotEventTopic`**
    <p>Notification received when an robot event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | event | Robot event type, see [RobotEvent.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Base_RobotEvent.md#). |
    | DeviceHandle | handle | Identifier of the hardware device connected or disconnected. |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the robot event. |
    | Connection | connection | Connection that caused the robot event. |

* **`/ServoingModeTopic`**
    <p>Notification received when an servoing mode event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | servoing_mode | New servoing mode, see [RobotEvent.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Base_ServoingMode.md#). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the servoing mode event. |
    | Connection | connection | Connection that caused the servoing mode event. |

* **`/FactoryTopic`**
    <p>Notification received when an factory event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | event | Factory event type, see [RobotEvent.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Base_FactoryEvent.md#). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the factory event. |
    | Connection | connection | Connection that caused the factory event. |

* **`/NetworkTopic`**
    <p>Notification received when an network event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | event | Network event type, see [RobotEvent.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Base_NetworkEvent.md#). |
    | Timestamp | timestamp | Event timestamp. |
    | UserProfileHandle | user_handle | User that caused the network event. |
    | Connection | connection | Connection that caused the network event. |

* **`/ArmStateTopic`**

    <p>Notification received when an armstate event occured.</p>

    | Type | Name | Description |
    |:---:|:---:|:---:|
    | uint32 | active_state | New arm state, see [ArmState.msg](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/enm_Common_ArmState.md#). |
    | Timestamp | timestamp | Event timestamp. |
    | Connection | connection | Connection that caused the network event. |

### Services
Most of the services supported by this node are generated from the [C++ Kortex API](https://github.com/Kinovarobotics/kortex). You can find the documentation [here](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/index.md).

Example:
If you want to call the ROS service **`GetActuatorCount`** that has been generated by the C++ method [GetActuatorCount](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/summary_Base.md), you would initialize your service like this:

    ros::ServiceClient client_GetActuatorCount = n.serviceClient<kortex_driver::GetActuatorCount>("GetActuatorCount");

#### Non-generated
* **`SetApiOptions`**
    <p>Modifies the Kortex API options. Once this service is called, the options set will affect every future call to the node.</p>

* **`SetDeviceID`**
    <p>Modifies the target device (device routing feature) of the node. The default value is 0.</p>

### Messages
Most of the messages supported by this node are generated from the [ C++ Kortex API](https://github.com/Kinovarobotics/kortex). You can find the documentation [here](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/index.md).

#### Non-generated
* **`ApiOptions`**
    <p>Represents all the option that you can set on the Kortex API.</p>

* **`KortexError`**
    <p>Represents a Kortex API error.</p>

### Protos files
The **protos** directory contains the protobuf files from where the MSG, SRV and source files are generated. The content of this folder should not be modified.

### Template files
The **templates** directory contains all the JINJA2 files needed by the protoc generator. For more details on the generation process, see the **Generation** section.

| Filename | Description |
|:---:|:---:|
| main.jinja2 | Used to generate src/main.cpp |
| NodeServices.cpp.jinja2 | Used to generate src/node.cpp |
| NodeServices.h.jinja2 | Used to generate src/node.h |
| proto_converterCPP.jinja2 | Used to generate every src/*_proto\_converter.cpp file |
| proto_converterHeader.jinja2 | Used to generate every src/*_proto\_converter.h file |
| ros_converterCPP.jinja2 | Used to generate every src/*_ros\_converter.cpp file |
| ros_converterHeader.jinja2 | Used to generate every src/*_proto\_converter.h file |
| ros_enum.jinja2 | Used to generate every msg/*.msg files that represent a protobuf enum |
| ros_message.jinja2 | Used to generate every msg/*.msg files that represent a protobuf message |
| ros_oneof.jinja2 | Used to generate every msg/*.msg files that represent a protobuf oneof |
| ros_service.jinja2 | Used to generate every msg/*.msg files that represent a protobuf RPC |

## Generation
<p>
The generation process is based on a custom protoc plugin. Basically, most of the generation process is in the RosGeneration.py file located in the package root directory. Before launching the generation ensure that you have the Python JINJA2 module installed.
</p>

To launch the generation of this package:

1. Open a terminal window.
1. Browse to the root directory of this package [YOUR\_ROS\_WORKSPACE]/src/ros\_kortex/kortex\_driver/
1. Ensure that the kortex_driver.sh file can be executed. If not then run: <code>chmod +x kortex_driver.sh</code>
1. Run the command: <code>protoc --plugin=protoc-gen-custom=kortex_driver.sh -I./protos/ --custom_out=./build ./protos/\*.prot</code>
1. The result of the generation should be in the following folders:
    * /src
    * /msg
    * /srv

