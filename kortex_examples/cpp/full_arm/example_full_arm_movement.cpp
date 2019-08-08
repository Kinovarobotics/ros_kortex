/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#include <thread>

#include "ros/ros.h"
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/PlayCartesianTrajectory.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/PlayJointTrajectory.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>

#define HOME_ACTION_IDENTIFIER 2

void example_clear_faults(ros::NodeHandle n, std::string robot_name)
{
  ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name + "/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults))
  {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void example_home_the_robot(ros::NodeHandle n, std::string robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // We can now execute the Action that we read 
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("The Home position action was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

void example_set_cartesian_reference_frame(ros::NodeHandle n, std::string robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

void example_send_cartesian_pose(ros::NodeHandle n, std::string robot_name)
{
  // Get the actual cartesian pose to increment it
  // You can create a subscriber to listen to the base_feedback
  // Here we only need the latest message in the topic though
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");

  // Initialize the ServiceClient for the cartesian pose
  ros::ServiceClient service_client_play_cartesian_trajectory = n.serviceClient<kortex_driver::PlayCartesianTrajectory>("/" + robot_name + "/base/play_cartesian_trajectory");
  kortex_driver::PlayCartesianTrajectory service_play_cartesian_trajectory;

  // Initialize input
  float current_x = feedback->base.commanded_tool_pose_x;
  float current_y = feedback->base.commanded_tool_pose_y;
  float current_z = feedback->base.commanded_tool_pose_z;
  float current_theta_x = feedback->base.commanded_tool_pose_theta_x;
  float current_theta_y = feedback->base.commanded_tool_pose_theta_y;
  float current_theta_z = feedback->base.commanded_tool_pose_theta_z;

  // Creating the target pose
  service_play_cartesian_trajectory.request.input.target_pose.x = current_x;
  service_play_cartesian_trajectory.request.input.target_pose.y = current_y;
  service_play_cartesian_trajectory.request.input.target_pose.z = current_z + 0.15;
  service_play_cartesian_trajectory.request.input.target_pose.theta_x = current_theta_x;
  service_play_cartesian_trajectory.request.input.target_pose.theta_y = current_theta_y;
  service_play_cartesian_trajectory.request.input.target_pose.theta_z = current_theta_z + 35;

  kortex_driver::CartesianSpeed poseSpeed;
  poseSpeed.translation = 0.1;
  poseSpeed.orientation = 15;

  // The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
  // To specify a one_of, create it and put it in the appropriate vector of the oneof_type member of the ROS object : 
  service_play_cartesian_trajectory.request.input.constraint.oneof_type.speed.push_back(poseSpeed);

  if (service_client_play_cartesian_trajectory.call(service_play_cartesian_trajectory))
  {
    ROS_INFO("The new cartesian pose was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call PlayCartesianTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

void example_send_joint_angles(ros::NodeHandle n, std::string robot_name, int degrees_of_freedom)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_play_joint_trajectory = n.serviceClient<kortex_driver::PlayJointTrajectory>("/" + robot_name + "/base/play_joint_trajectory");
  kortex_driver::PlayJointTrajectory service_play_joint_trajectory;

  std::vector<double> angles_to_send;
  for (unsigned int i = 0; i < degrees_of_freedom; i++)
  {
    angles_to_send.push_back(0.0);
  }

  for (int i = 0; i < degrees_of_freedom; i++)
  {
    kortex_driver::JointAngle temp_angle;
    temp_angle.joint_identifier = i;
    temp_angle.value = angles_to_send[i];
    service_play_joint_trajectory.request.input.joint_angles.joint_angles.push_back(temp_angle);
  }

  if (service_client_play_joint_trajectory.call(service_play_joint_trajectory))
  {
    ROS_INFO("The joint angles were sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call PlayJointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

void example_send_gripper_command(ros::NodeHandle n, std::string robot_name, double value)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/" + robot_name + "/base/send_gripper_command");
  kortex_driver::SendGripperCommand service_send_gripper_command;

  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  service_send_gripper_command.request.input.gripper.finger.push_back(finger);
  service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

  if (service_client_send_gripper_command.call(service_send_gripper_command))  
  {
    ROS_INFO("The gripper command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call SendGripperCommand";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "full_arm_movement_example_cpp");

  //*******************************************************************************
  // ROS Parameters
  ros::NodeHandle n;
  std::string robot_name = "my_gen3";
  int degrees_of_freedom = 7;
  bool is_gripper_present = false;

  // Parameter robot_name
  if (!ros::param::get("~robot_name", robot_name))
  {
    std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name + " as namespace";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using robot_name " + robot_name + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter degrees_of_freedom
  if (!ros::param::get("/" + robot_name + "/degrees_of_freedom", degrees_of_freedom))
  {
    std::string error_string = "Parameter /" + robot_name + "/degrees_of_freedom was not specified, defaulting to " + std::to_string(degrees_of_freedom) + " as degrees of freedom";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using degrees_of_freedom " + std::to_string(degrees_of_freedom) + " as degrees_of_freedom";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter is_gripper_present
  if (!ros::param::get("/" + robot_name + "/is_gripper_present", is_gripper_present))
  {
    std::string error_string = "Parameter /" + robot_name + "/is_gripper_present was not specified, defaulting to " + std::to_string(is_gripper_present);
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using is_gripper_present " + std::to_string(is_gripper_present);
    ROS_INFO("%s", error_string.c_str());
  }
  //*******************************************************************************

  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in fault
  example_clear_faults(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Move the robot to the Home position with an Action
  example_home_the_robot(n, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  //*******************************************************************************

  //*******************************************************************************
  // Example of gripper command
  // Let's close the gripper
  if (is_gripper_present)
  {
    example_send_gripper_command(n, robot_name, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));  
  }
  //*******************************************************************************

  //*******************************************************************************
  // Set the reference frame to "Mixed"
  example_set_cartesian_reference_frame(n, robot_name);

  // Example of cartesian pose
  // Let's make it move in Z
  example_send_cartesian_pose(n, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  //*******************************************************************************

  //*******************************************************************************
  // Example of angular position
  // Let's send the arm to vertical position
  example_send_joint_angles(n, robot_name, degrees_of_freedom);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  //*******************************************************************************

  //*******************************************************************************
  // Example of gripper command
  // Let's close the gripper
  if (is_gripper_present)
  {
    example_send_gripper_command(n, robot_name, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));  
  }
  //*******************************************************************************

  return 0;
}