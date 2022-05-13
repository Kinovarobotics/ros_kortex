#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
# mkdir catkin_ws
# mkdir catkin_ws/src
# ln -s /ros_kortex /catkin_ws/src/ros_kortex
# ln -s /ros_kortex_vision /catkin_ws/src/ros_kortex_vision
cd /catkin_ws
# catkin_make
source /catkin_ws/devel/setup.bash
exec "$@"