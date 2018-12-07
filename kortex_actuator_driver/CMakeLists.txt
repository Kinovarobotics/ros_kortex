
cmake_minimum_required(VERSION 2.8.3)
project(kortex_actuator_driver)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

file(GLOB_RECURSE cpp_list RELATIVE ${PROJECT_SOURCE_DIR} "src/*.cpp")

## Declare ROS messages and services
add_message_files(DIRECTORY msg)
add_message_files(DIRECTORY msg/non_generated)
add_service_files(DIRECTORY srv)
add_service_files(DIRECTORY srv/non_generated)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
include_directories(include ${PROJECT_SOURCE_DIR}/src/util)

link_directories(${PROJECT_SOURCE_DIR}/../kortex_api/lib/release)

add_executable(${PROJECT_NAME} ${cpp_list})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} CppKinovaApi gcov)

find_package(Protobuf 3.5.1 REQUIRED)

if(Protobuf_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${PROTOBUF_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
else()
    message(FATAL_ERROR "Protobuf libraries not found")
endif()

add_dependencies(${PROJECT_NAME} kortex_actuator_driver_gencpp)

