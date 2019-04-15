# ROS KORTEX
The official ROS package to interact with Kortex and its related products is built upon the Kortex API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).

## Content
### kortex_bringup
This package contains the launch files that start the nodes and load the correct parameters and robot description to the Parameter Server.
For more details, please consult the [README](kortex_bringup/readme.md)

### kortex_driver
This package implements a ROS node that allows communication between a node and a Gen3 robot. For more details, please consult the [README](kortex_driver/readme.md) from in the package subdirectory. Use this package if you want to:

* Change basic robot configuration.
* Move the robot in Cartesian space.
* Move the robot in joint space.
* Activate admittance mode.
* Move the robot using **LOW\_LEVEL**\ (1 kHz\) control mode.
* Move the robot using **LOW\_LEVEL\_BYPASS**\ mode.
* Access cyclic data sporadically.


### kortex_actuator_driver
This package implements a ROS node that allows direct communication with a Gen3 actuator. Direct communication means that either the computer running the node is directly connected to the actuator or that it is connected to a robot using the device routing system. A more detailed [description](kortex_actuator_driver/readme.md) can be found in the package subdirectory. Use this package if you would like to:

* Change an advance configuration setting on an actuator.
* Move an actuator using the cyclic data (1 kHz).

### kortex_device_manager
This package implements a ROS node that allows basic communication with every device supported by the Kortex framework. A more detailed [description](kortex_device_manager/readme.md) can be found in the package subdirectory. Use this package if you would like to:

* List all devices available on a specific Gen3 robot.
* Retrieve generic information of a given device.
* Get the firmware version of a given device.
* Get the serial number of a given device.
* Set IPv4 settings on a given device.
* Get safety information of a given device.

### kortex_vision_config_driver
This package implements a ROS node that allows direct communication with a Gen3 Vision module. Direct communication means that either the computer running the node has an Ethernet cable directly connected to a Vision module or that it is connected to a robot using the device routing system. A more detailed [description](kortex_vision_config_driver/readme.md) can be found in the package subdirectory. Use this package if you would like to:

* Change a configuration setting on a vision module.
* Get informations about the configuration settings of a vision module.

### kortex_examples
This package holds all the examples needed to understands that basics of ros_kortex. All examples are written in both c++ and python. A more detailed [description](kortex_examples/readme.md) can be found in the package subdirectory.

### kortex_description
This package contains the URDF and the STL of a complete Gen3 robot. A more detailed [description](kortex_description/readme.md) can be found in the package directory.

### kortex_api
This package contains all the header files and the libraries required by the Kortex C++ API. A more detailed [description](kortex_api/readme.md) can be found in the package subdirectory.

## Setup
### Install protobuf
The protobuf compiler is required if you need to re-generate the original package code.

1. git clone https://github.com/protocolbuffers/protobuf --branch 3.5.1.1   (you must use this specific version)
2. Follow these [instructions](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md) to build and install protobuf and its compiler. 

### Download the API and extract it to the kortex_api/include and kortex_api/lib folders
The build.sh script automates this process. To launch it, go to the root of your catkin_workspace and execute :
```./src/ros_kortex/build.sh```
This will download the API, unzip it to the right folder and build the project with ```catkin_make```.

You can also download and unzip the API manually by following these [instructions](kortex_api/readme.md).

## kortex_gazebo
This package is not completed yet but will be available in a future version.

## kortex_moveit_config
This package is not completed yet but will be available in a future version.
