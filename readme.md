# ros_kortex
ROS Kortex is the official ROS package to interact with Kortex and its related products. It is built upon the Kortex API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).

## Installation

### Setup

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

You can find the instructions to install ROS Kinetic (for Ubuntu 16.04) [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

[Google Protocol Buffers](https://developers.google.com/protocol-buffers/) is used by Kinova to define the Kortex APIs and to automatically generate ROS messages, services and C++ classes from the Kortex API `.proto` files. The installation of Google Protocol Buffers is required by developers implementing new APIs with the robot. However, since we already provide all the necessary generated files on GitHub, this is not required for most end users of the robot.

If you have a specific use case that requires you to install it, you can follow the instructions provided [at the end of this readme file](#protobuf-installation). We recommend that you contact Kinova if you have any specific questions about this.


### Build

These are the instructions to run in a terminal to create the workspace, clone the `ros_kortex` repository, install the necessary ROS dependencies and build the package:

        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone https://github.com/Kinovarobotics/ros_kortex.git
        cd ../
        rosdep install --from-paths src --ignore-src
        catkin_make
        source devel/setup.bash

## Contents

The following is a description of the packages included in this repository.

### kortex_api
This package contains all the header files and the libraries of the C++ Kortex API. The files are automatically downloaded from the Web and extracted when you `catkin_make` if the `kortex_api/include` and `kortex_api/lib` folders are empty. 

**Note:**  Upon a new release of the API, it is important to delete the content of these two folders to make sure the new API gets downloaded and you don't get build errors when you `catkin_make`.

A more detailed [description](kortex_api/readme.md) can be found in the package subdirectory.

### kortex_control
This package implements the simulation controllers that control the arm in Gazebo. For more details, please consult the [README](kortex_control/readme.md) from the package subdirectory.

**Note** The `ros_control` controllers for the real arm are not yet implemented and will be in a future release of `ros_kortex`.

### kortex_description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots. For more details, please consult the [README](kortex_description/readme.md) from the package subdirectory.

### kortex_driver
This package implements a ROS node that allows communication between a node and a Kinova Gen3 Ultra lightweight robot. For more details, please consult the [README](kortex_driver/readme.md) from the package subdirectory.

### kortex_examples
This package holds all the examples needed to understand the basics of `ros_kortex`. Most of the examples are written in both C++ and Python. Only the MoveIt! example is available exclusively in Python for now.
A more detailed [description](kortex_examples/readme.md) can be found in the package subdirectory.

### kortex_gazebo
This package contains files to simulate the Kinova Gen3 Ultra lightweight robot in Gazebo. For more details, please consult the [README](kortex_gazebo/readme.md) from the package subdirectory.

### kortex_move_it_config
This metapackage contains the auto-generated MoveIt! files to use the Kinova Gen3 arm with the MoveIt! motion planning framework. For more details, please consult the [README](kortex_move_it_config/readme.md) from the package subdirectory.

### third_party
This folder contains the third-party packages we use with the ROS Kortex packages. Currently, it consists of two packages used for the simulation of the Robotiq Gripper in Gazebo. We use [gazebo-pkgs](third_party/gazebo-pkgs/README.md) for grasping support in Gazebo and [roboticsgroup_gazebo_plugins](third_party/roboticsgroup_gazebo_plugins/README.md) to mimic joint support in Gazebo.

<a id="protobuf-installation"></a>
## Instructions to install Protocol Buffers (optional)

You can clone the Protocol Buffers repository from GitHub with this command: 

```cpp
git clone https://github.com/protocolbuffers/protobuf --branch 3.5.1.1   (you must use this specific version)
```
You can install Protocol Buffers by following these [instructions](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md).
