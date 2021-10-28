cmake_minimum_required(VERSION 3.0.2)

project(kortex_gazebo)

# This is added to remove policy CMP0054 warning (see https://stackoverflow.com/questions/45900159/how-to-use-variables-and-avoid-cmp0054-policy-violations)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED)

find_package(gazebo)

catkin_package()

# Install
install(PROGRAMS
   scripts/home_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.launch"
)

# Gazebo is compiled with Protobuf 2.6.1 and Kortex with Protobuf 3.5.1
# The 2.6.1 includes are prepended to the include_directories so the Gazebo plugins we compile here build
include_directories(BEFORE ./include/)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# To add your Gazebo plugin to this package, put your source file under src and uncomment/modify those lines
# add_library(my_gazebo_plugin SHARED src/my_gazebo_plugin.cpp)
# target_link_libraries(my_gazebo_plugin ${GAZEBO_LIBRARIES})
