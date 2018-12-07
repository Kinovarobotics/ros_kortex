
cmake_minimum_required(VERSION 2.8.3)
project(kortex_examples)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation kortex_driver kortex_actuator_driver kortex_vision_config_driver kortex_device_manager)

file(GLOB_RECURSE cpp_list RELATIVE ${PROJECT_SOURCE_DIR} "cpp/*.cpp")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
include_directories(include ${PROJECT_SOURCE_DIR}/src/util)

add_executable(PlayCartesian cpp/example_play_cartesian.cpp)
target_link_libraries(PlayCartesian ${catkin_LIBRARIES} )

add_executable(PlayCartesianPosition cpp/example_play_cartesian_position.cpp)
target_link_libraries(PlayCartesianPosition ${catkin_LIBRARIES} )

add_executable(GetControlLoopParameters cpp/example_get_control_loop_parameters.cpp)
target_link_libraries(GetControlLoopParameters ${catkin_LIBRARIES} )

add_executable(SetControlLoopParameters cpp/example_set_control_loop_parameters.cpp)
target_link_libraries(SetControlLoopParameters ${catkin_LIBRARIES} )

add_executable(GetSensorSettings cpp/example_get_sensor_settings.cpp)
target_link_libraries(GetSensorSettings ${catkin_LIBRARIES} )

add_executable(ReadAllDevices cpp/example_read_all_devices.cpp)
target_link_libraries(ReadAllDevices ${catkin_LIBRARIES} )
