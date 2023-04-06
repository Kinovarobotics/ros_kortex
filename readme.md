# ros_kortex
ROS Kortex is the official ROS package to interact with Kortex and its related products. It is built upon the Kortex API, documentation for which can be found in the [GitHub Kortex repository](https://github.com/Kinovarobotics/kortex).

## Download links

You can refer to the [Kortex repository "Download links" section](https://github.com/Kinovarobotics/kortex#download-links) to download the firmware package and the release notes.

### Accessing the color and depth streams 

To access the color and depth streams, you will need to clone and follow the instructions to install the [ros_kortex_vision repository ](https://github.com/Kinovarobotics/ros_kortex_vision).
## Installation

### Setup

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

This package has been tested under ROS Kinetic (Ubuntu 16.04) and ROS Melodic (Ubuntu 18.04).
You can find the instructions to install ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and ROS Melodic [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

[Google Protocol Buffers](https://developers.google.com/protocol-buffers/) is used by Kinova to define the Kortex APIs and to automatically generate ROS messages, services and C++ classes from the Kortex API `.proto` files. The installation of Google Protocol Buffers is required by developers implementing new APIs with the robot. However, since we already provide all the necessary generated files on GitHub, this is not required for most end users of the robot.

### Build

These are the instructions to run in a terminal to create the workspace, clone the `ros_kortex` repository and install the necessary ROS dependencies:

        sudo apt install python3 python3-pip
        sudo python3 -m pip install conan==1.59
        conan config set general.revisions_enabled=1
        conan profile new default --detect > /dev/null
        conan profile update settings.compiler.libcxx=libstdc++11 default
        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone -b <branch-name> https://github.com/Kinovarobotics/ros_kortex.git
        cd ../
        rosdep install --from-paths src --ignore-src -y

> `<branch-name>` corresponds to the branch matching your ROS version (noetic-devel, melodic-devel, kinetic-devel)

> Instructions are for conan V1.X only and it won't work for versions >=2.0.0

Then, to build and source the workspace:

        catkin_make
        source devel/setup.bash

You can also build against one of the ARMv8 builds of the Kortex API with Conan if you specify the `CONAN_TARGET_PLATFORM` CMake argument when using `catkin_make`. The following platforms are supported:

- Artik 710: 

        catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=artik710
        source devel/setup.bash

- IMX6:

        catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=imx6
        source devel/setup.bash

- NVidia Jetson: 

        catkin_make --cmake-args -DCONAN_TARGET_PLATFORM=jetson
        source devel/setup.bash

As you see, there are instructions to install the Conan package manager. You can learn more about why we use Conan or how to simply download the API and link against it [in this specific section of the kortex_driver readme](kortex_driver/readme.md#conan). You can also decide 

<p><details close>
<summary>Conan SSL Error</summary>

While running `catkin_make`, you may get a SSL Certificate error similar to this

```sh
ERROR: HTTPSConnectionPool(host='artifactory.kinovaapps.com', port=443): Max retries exceeded with url: /artifactory/api/conan/conan/v1/ping (Caused by SSLError(SSLCertVerificationError(1, '[SSL: CERTIFICATE_VERIFY_FAILED] certificate verify failed: certificate has expired (_ssl.c:1131)')))
```

This is because Conan's root certificate expired on 2021-09-30

You can fix this by running

```sh
conan config install https://github.com/conan-io/conanclientcert.git
```

 </details></p>

## Contents

The following is a description of the packages included in this repository.

### kortex_control
This package implements the simulation controllers that control the arm in Gazebo. For more details, please consult the [README](kortex_control/readme.md) from the package subdirectory.

**Note** The `ros_control` controllers for the real arm are not yet implemented and will be in a future release of `ros_kortex`.

### kortex_description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots. For more details, please consult the [README](kortex_description/readme.md) from the package subdirectory.

### kortex_driver
This package implements a ROS node that allows communication between a node and a Kinova Gen3 or Gen3 lite robot. For more details, please consult the [README](kortex_driver/readme.md) from the package subdirectory.

### kortex_examples
This package holds all the examples needed to understand the basics of `ros_kortex`. Most of the examples are written in both C++ and Python. Only the MoveIt! example is available exclusively in Python for now.
A more detailed [description](kortex_examples/readme.md) can be found in the package subdirectory.

### kortex_gazebo
This package contains files to simulate the Kinova Gen3 and Gen3 lite robots in Gazebo. For more details, please consult the [README](kortex_gazebo/readme.md) from the package subdirectory.

### kortex_move_it_config
This metapackage contains the auto-generated MoveIt! files to use the Kinova Gen3 and Gen3 lite arms with the MoveIt! motion planning framework. For more details, please consult the [README](kortex_move_it_config/readme.md) from the package subdirectory.

### third_party
This folder contains the third-party packages we use with the ROS Kortex packages. Currently, it consists of two packages used for the simulation of the Robotiq Gripper in Gazebo. We use [gazebo-pkgs](third_party/gazebo-pkgs/README.md) for grasping support in Gazebo and [roboticsgroup_gazebo_plugins](third_party/roboticsgroup_gazebo_plugins/README.md) to mimic joint support in Gazebo.
