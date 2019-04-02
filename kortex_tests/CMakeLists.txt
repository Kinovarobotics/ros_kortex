
cmake_minimum_required(VERSION 2.8.3)
project(kortex_tests)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation kortex_driver kortex_actuator_driver kortex_vision_config_driver kortex_device_manager)

file(GLOB_RECURSE cpp_list RELATIVE ${PROJECT_SOURCE_DIR} "*.cpp")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)


add_executable(kortex_tests_main src/main.cpp)
add_dependencies(kortex_tests_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(kortex_tests_main ${catkin_LIBRARIES} )


