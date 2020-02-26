#!/bin/bash

# This is a helper script that sources ROS, cleans and builds the catkin workspace and the tests, and installs the missing rosdep
# It is meant to be run from the root of the workspace
rosdep update
apt-get update
rosdep -y install --from-paths src --ignore-src
rm -rf devel/ build/ 
catkin_make
catkin_make tests
echo 'Finished building the package!'
exit 0
