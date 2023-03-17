cmake_minimum_required(VERSION 2.8.3)
project(kortex_driver)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

option(USE_CONAN "Use the Conan package manager to automatically fetch the Kortex API" ON)

if(NOT CONAN_TARGET_PLATFORM)
  set(CONAN_TARGET_PLATFORM "x86")
endif()
message("CONAN_TARGET_PLATFORM is ${CONAN_TARGET_PLATFORM}")

# Explicitely specify the build type in case the user did not do it
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs controller_manager_msgs message_generation actionlib control_msgs urdf moveit_ros_planning_interface)
find_package(Boost REQUIRED COMPONENTS system)

file(GLOB_RECURSE generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/generated/robot/*.cpp" "src/generated/simulation/*.cpp")
file(GLOB_RECURSE non_generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/driver/*.cpp")
file(GLOB_RECURSE test_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/tests/*.cc")

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

add_action_files(DIRECTORY action/non_generated)

## generate added messages and services
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## declare a catkin package
catkin_package(CATKIN_DEPENDS actionlib_msgs)

function(get_closest_build_type VAR_OUT)
  if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(${VAR_OUT} "Debug" PARENT_SCOPE)
    return()
  endif()
  string(TOUPPER ${CMAKE_BUILD_TYPE} BTYPE)
  if("${BTYPE}" STREQUAL "DEBUG")
    set(${VAR_OUT} "Debug" PARENT_SCOPE)
  else()
    set(${VAR_OUT} "Release" PARENT_SCOPE)
  endif()
endfunction()

if(USE_CONAN)
  # Include conan.cmake module and download Kortex API from artifactory
  include(${PROJECT_SOURCE_DIR}/cmake/conan.cmake)
  conan_check(REQUIRED)
  conan_add_remote(NAME kinova_public
                  URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-public)
  get_closest_build_type(CONAN_BUILD_TYPE)
  if("${CONAN_TARGET_PLATFORM}" STREQUAL "x86")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=5 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE})
  elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "artik710")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=5 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE}
                    ENV TARGET=artik710)
  elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "imx6")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=6.4 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE}
                    ENV TARGET=imx6)
  elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "jetson")
    conan_cmake_run(CONANFILE conanfile.py
                    UPDATE
                    BASIC_SETUP CMAKE_TARGETS
                    NO_OUTPUT_DIRS
                    SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=7 kortex_api_cpp:compiler.libcxx=libstdc++11 kortex_api_cpp:build_type=${CONAN_BUILD_TYPE}
                    ENV TARGET=jetson)
  endif()
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/include)

if(NOT USE_CONAN)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
  include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include)
  link_directories(${PROJECT_SOURCE_DIR}/../kortex_api/lib/release)
endif()

# Generated files library
add_library(kortex_driver_generated_files ${generated_files})
if(USE_CONAN)
  target_link_libraries(kortex_driver_generated_files CONAN_PKG::kortex_api_cpp ${catkin_LIBRARIES} gcov)
else()
  target_link_libraries(kortex_driver_generated_files KortexApiCpp ${catkin_LIBRARIES} gcov)
endif()
add_dependencies(kortex_driver_generated_files ${PROJECT_NAME}_gencpp)
# Remove deprecation warnings from Kortex API
target_compile_options(kortex_driver_generated_files PRIVATE "-Wno-deprecated-declarations")

# Non-generated files library (driver implementation)
add_library(kortex_arm_driver_implementation ${non_generated_files})
if(USE_CONAN)
  target_link_libraries(kortex_arm_driver_implementation CONAN_PKG::kortex_api_cpp ${catkin_LIBRARIES} gcov kortex_driver_generated_files)
else()
  target_link_libraries(kortex_arm_driver_implementation KortexApiCpp ${catkin_LIBRARIES} gcov kortex_driver_generated_files)
endif()
add_dependencies(kortex_arm_driver_implementation kortex_driver_generated_files)

# Entry point executable for the driver
add_executable(kortex_arm_driver src/non-generated/main.cpp)
if(USE_CONAN)
  target_link_libraries(kortex_arm_driver CONAN_PKG::kortex_api_cpp ${catkin_LIBRARIES} gcov kortex_arm_driver_implementation)
else()
  target_link_libraries(kortex_arm_driver KortexApiCpp ${catkin_LIBRARIES} gcov kortex_arm_driver_implementation)
endif()
add_dependencies(kortex_arm_driver kortex_arm_driver_implementation)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	catkin_add_gtest(kortex_arm_driver_func_tests ${test_files})
  target_link_libraries(kortex_arm_driver_func_tests ${catkin_LIBRARIES} kortex_arm_driver_implementation)
  add_dependencies(kortex_arm_driver_func_tests kortex_arm_driver_implementation)
endif()
