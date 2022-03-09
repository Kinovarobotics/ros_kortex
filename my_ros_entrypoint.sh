#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
mkdir pkg/catkin_ws
mkdir pkg/catkin_ws/src
cd pkg/catkin_ws
catkin_make
#ln -s /pkg/ros_kortex /pkg/catkin_ws/src
#catkin_make
source devel/setup.bash
exec "$@"