
cmake_minimum_required(VERSION 2.8.3)
project(kortex_driver)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# custom bash script to download the API if it has not been downloaded yet
execute_process(COMMAND ./download_kortex_api.bash
                WORKING_DIRECTORY ../src/ros_kortex/kortex_api/scripts
                RESULT_VARIABLE DOWNLOAD_API_RESULT)

if(NOT DOWNLOAD_API_RESULT EQUAL 0)
  message(FATAL_ERROR "Download of the Kortex API failed")
endif()

## find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation actionlib control_msgs)
find_package(Boost REQUIRED COMPONENTS system)

file(GLOB_RECURSE generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/generated/*.cpp")
file(GLOB_RECURSE non_generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/*.cpp")

# Find all auto-generated subdirectories in msg/generated
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/msg/generated ${PROJECT_SOURCE_DIR}/msg/generated/*)
set(msg_generated_dir_list "")
foreach(child ${children})
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/msg/generated/${child})
      list(APPEND msg_generated_dir_list ${child})
    endif()
endforeach()

# Find all auto-generated subdirectories in srv/generated
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/srv/generated ${PROJECT_SOURCE_DIR}/srv/generated/*)
set(srv_generated_dir_list "")
foreach(child ${children})
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/srv/generated/${child})
      list(APPEND srv_generated_dir_list ${child})
    endif()
endforeach()

## declare ROS messages and services
add_message_files(DIRECTORY msg/non_generated)
add_message_files(DIRECTORY msg/generated)
foreach(sub_dir ${msg_generated_dir_list})
    add_message_files(DIRECTORY msg/generated/${sub_dir})
endforeach()

add_service_files(DIRECTORY srv/non_generated)
foreach(sub_dir ${srv_generated_dir_list})
    add_service_files(DIRECTORY srv/generated/${sub_dir})
endforeach()

## generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/include)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include)

link_directories(${PROJECT_SOURCE_DIR}/../kortex_api/lib/release)

add_library(kortex_driver_generated_files ${generated_files})
target_link_libraries(kortex_driver_generated_files ${catkin_LIBRARIES} KortexApi gcov)
add_dependencies(kortex_driver_generated_files ${PROJECT_NAME}_gencpp)

add_executable(kortex_arm_driver ${non_generated_files})
target_link_libraries(kortex_arm_driver ${catkin_LIBRARIES} KortexApi gcov kortex_driver_generated_files)
add_dependencies(kortex_arm_driver kortex_driver_generated_files)
