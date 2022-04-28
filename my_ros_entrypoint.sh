#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
mkdir catkin_ws
mkdir catkin_ws/src
cd catkin_ws
ln -s /ros_kortex /catkin_ws/src
ln -s /ros_kortex_vision /catkin_ws/src
catkin_make
source devel/setup.bash
exec "$@"