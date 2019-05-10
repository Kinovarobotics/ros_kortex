
cmake_minimum_required(VERSION 2.8.3)
project(kortex_vision_config_driver)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

file(GLOB_RECURSE cpp_list RELATIVE ${PROJECT_SOURCE_DIR} "src/*.cpp")

## declare ROS messages and services
add_message_files(DIRECTORY msg)
add_message_files(DIRECTORY msg/non_generated)
add_service_files(DIRECTORY srv)
add_service_files(DIRECTORY srv/non_generated)

## generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
include_directories(include ${PROJECT_SOURCE_DIR}/src/util)

link_directories(${PROJECT_SOURCE_DIR}/../kortex_api/lib/release)

add_executable(${PROJECT_NAME} ${cpp_list})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} KortexApi gcov)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
