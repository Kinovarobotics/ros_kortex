#!/bin/bash

# This is a helper script that sources ROS, cleans and builds the catkin workspace
# It is meant to be run from the root of the workspace
# Can be used by end users, but mainly used because Docker (for CI) needs en executable file and cannot take multiple bash commands

source /opt/ros/kinetic/setup.bash
rm -rf devel/ build/ 
catkin_make -DGENERATE_PROTO_ROS_WRAPPER=OFF
