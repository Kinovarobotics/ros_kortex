# ROS KORTEX
The official ROS package to interact with Kortex and its related products. It is built on top of the Kortex API. Documentation of the Kortex API can be found in the Kortex [repository](https://github.com/Kinovarobotics/kortex).


<!-- MarkdownTOC -->

1. [Content](#content)
    1. [kortex driver](#kortex-driver)
    1. [kortex actuator](#kortex-actuator)
    1. [kortex device manager](#kortex-device-manager)
    1. [kortex vision module driver](#kortex-vision-module-driver)
    1. [kortex examples](#kortex-examples)
    1. [kortex description](#kortex-description)
    1. [kortex api](#kortex-api)
1. [Setup](#setup)
    1. [Install protobuf](#install-protobuf)
1. [kortex gazebo](#kortex-gazebo)
1. [kortex moveit](#kortex-moveit)

<!-- /MarkdownTOC -->

<a id="content"></a>
## Content
<a id="kortex-driver"></a>
### kortex driver
This package is a ROS node that allows communication with a robotic base from a Gen3 robot. For more details, please read the [documentation](kortex_driver/readme.md) from inside the package. Use this package if you want to:

* Change some basic configuration on the robot.
* Move the robot in the Cartesian space.
* Move the robot in the Joint space.
* Activate the admittance mode.
* Move the robot using the **LOW\_LEVEL**\ (1 KHz\) control mode.
* Move the robot using the **LOW\_LEVEL\_BYPASS**\ mode.
* Access the cyclic data sporadically.

<a id="kortex-actuator"></a>
### kortex actuator
This package is a ROS node that allows a direct communication with a Gen3 actuator. A direct communication means that either the computer that is running the node has direct IP connectivity with the actuator or is connected to a robot and using the device routing system to reach the actuator. A more detailed [documentation](kortex_actuator_driver/readme.md) can be found inside the package. Use this package if you want to:

* Change an advance configuration setting on an actuator.
* Move an actuator using the cyclic data (1 KHz).

<a id="kortex-device-manager"></a>
### kortex device manager
This package is a ROS node that allows basic communication with every devices supported by the kortex framework. A more detailed [documentation](kortex_device_manager/readme.md) can be found inside the package. Use this package if you want to:

* List all the devices available on a specific Gen3 robot.
* Retrieve some generic informations on a specific device.
* Get the firmware version of a specific device.
* Get the serial number of a device.
* Set IPv4 settings on a device.
* Get safety informations of a device.

<a id="kortex-vision-module-driver"></a>
### kortex vision module driver
This package is a ROS node that allows direct communication with a Gen3 vision module. A direct communication means that either the computer running the node has an ethernet cable directly connected to a vision module or that it is connected to a robot using the device routing system. A more detailed [documentation](kortex_vision_config_driver/readme.md) can be found inside the package. Use this package if you want to:

* Change a configuration setting on a vision module.
* Get informations about the configuration settings of a vision module.

<a id="kortex-examples"></a>
### kortex examples
This package holds all the examples needed to understands that basics of ros_kortex. All examples are written in both c++ and python. A more detailed [documentation](kortex_examples/readme.md) can be found inside the package.

<a id="kortex-description"></a>
### kortex description
This package contains the URDF and the STL of a complete Gen3 robot. A more detailed [documentation](kortex_description/readme.md) can be found inside the package.

<a id="kortex-api"></a>
### kortex api
This package contains all the header files and the libraries needed by the kortex C++ API. A more detailed [documentation](kortex_api/readme.md) can be found inside the package.

<a id="setup"></a>
## Setup
<a id="install-protobuf"></a>
### Install protobuf
Protobuf compiler is needed if you want to re-generate the code of a package.

1. git clone https://github.com/protocolbuffers/protobuf --branch 3.5.1.1   (you absolutely need to use this specific version)
2. Follow these [instructions](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md) to build and install protobuf and its compiler. 

<a id="kortex-gazebo"></a>
## kortex gazebo
This package is not completed yet but will be available in a future version.

<a id="kortex-moveit"></a>
## kortex moveit
This package is not completed yet but will be available in a future version.
