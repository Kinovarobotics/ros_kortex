
cmake_minimum_required(VERSION 2.8.3)
project(kortex_examples)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation kortex_driver)

## Declare a catkin package
catkin_package()
catkin_python_setup()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)

add_executable(example_actuator_configuration_cpp cpp/actuator_config/example_actuator_configuration.cpp)
add_dependencies(example_actuator_configuration_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_actuator_configuration_cpp ${catkin_LIBRARIES} )

add_executable(example_vision_configuration_cpp cpp/vision_config/example_vision_configuration.cpp)
add_dependencies(example_vision_configuration_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_vision_configuration_cpp ${catkin_LIBRARIES} )

add_executable(example_full_arm_movement_cpp cpp/full_arm/example_full_arm_movement.cpp)
add_dependencies(example_full_arm_movement_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_full_arm_movement_cpp ${catkin_LIBRARIES} )
