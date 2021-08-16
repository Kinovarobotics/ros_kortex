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
#include <atomic>

#include "ros/ros.h"

#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/ValidateWaypointList.h>
#include <kortex_driver/GetProductConfiguration.h>
#include <kortex_driver/ModelId.h>

#define HOME_ACTION_IDENTIFIER 2

std::atomic<int> last_action_notification_event{0};

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
}

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification");
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification");
      return false;
    }
    ros::spinOnce();
  }
  return false;
}

kortex_driver::Waypoint FillCartesianWaypoint(float new_x, float new_y, float new_z, float new_theta_x, float new_theta_y, float new_theta_z, float blending_radius)
{
  kortex_driver::Waypoint waypoint;
  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = new_x;
  cartesianWaypoint.pose.y = new_y;
  cartesianWaypoint.pose.z = new_z;
  cartesianWaypoint.pose.theta_x = new_theta_x;
  cartesianWaypoint.pose.theta_y = new_theta_y;
  cartesianWaypoint.pose.theta_z = new_theta_z;
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = blending_radius;

  waypoint.oneof_type_of_waypoint.cartesian_waypoint.push_back(cartesianWaypoint);

  return waypoint;
}

bool example_clear_faults(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name + "/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults))
  {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool example_home_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;
  last_action_notification_event = 0;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
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
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_set_cartesian_reference_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}

bool example_send_cartesian_pose(ros::NodeHandle n, const std::string &robot_name)
{
  last_action_notification_event = 0;
  // Get the actual cartesian pose to increment it
  // You can create a subscriber to listen to the base_feedback
  // Here we only need the latest message in the topic though
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");

  // Initialize the ServiceClient
  ros::ServiceClient service_client_execute_waypoints_trajectory = n.serviceClient<kortex_driver::ExecuteWaypointTrajectory>("/" + robot_name + "/base/execute_waypoint_trajectory");
  kortex_driver::ExecuteWaypointTrajectory service_execute_waypoints_trajectory;

  kortex_driver::Waypoint waypoint;

  // Initialize input
  float current_x = feedback->base.commanded_tool_pose_x;
  float current_y = feedback->base.commanded_tool_pose_y;
  float current_z = feedback->base.commanded_tool_pose_z;
  float current_theta_x = feedback->base.commanded_tool_pose_theta_x;
  float current_theta_y = feedback->base.commanded_tool_pose_theta_y;
  float current_theta_z = feedback->base.commanded_tool_pose_theta_z;

  // Creating the target pose
  service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(current_x, current_y,  current_z + 0.10, current_theta_x, current_theta_y, current_theta_z, 0));

  service_execute_waypoints_trajectory.request.input.duration = 0;
  service_execute_waypoints_trajectory.request.input.use_optimal_blending = false;

  if (service_client_execute_waypoints_trajectory.call(service_execute_waypoints_trajectory))
  {
    ROS_INFO("The new cartesian pose was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_send_joint_angles(ros::NodeHandle n, const std::string &robot_name, int degrees_of_freedom)
{
  last_action_notification_event = 0;
  // Initialize the ServiceClient
  ros::ServiceClient service_client_execute_waypoints_trajectory = n.serviceClient<kortex_driver::ExecuteWaypointTrajectory>("/" + robot_name + "/base/execute_waypoint_trajectory");
  kortex_driver::ExecuteWaypointTrajectory service_execute_waypoints_trajectory;

  ros::ServiceClient service_client_validate_waypoint_list = n.serviceClient<kortex_driver::ValidateWaypointList>("/" + robot_name + "/base/validate_waypoint_list");
  kortex_driver::ValidateWaypointList service_validate_waypoint_list;

  kortex_driver::WaypointList trajectory;
  kortex_driver::Waypoint waypoint;
  kortex_driver::AngularWaypoint angularWaypoint;

   // Angles to send the arm to vertical position (all zeros)
  for (unsigned int i = 0; i < degrees_of_freedom; i++)
  {
    angularWaypoint.angles.push_back(0.0);
  }

  // Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
  // If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
  int angular_duration = 0;
  angularWaypoint.duration = angular_duration;

  // Initialize Waypoint and WaypointList
  waypoint.oneof_type_of_waypoint.angular_waypoint.push_back(angularWaypoint);
  trajectory.duration = 0;
  trajectory.use_optimal_blending = false;
  trajectory.waypoints.push_back(waypoint);

  service_validate_waypoint_list.request.input = trajectory;
  if (!service_client_validate_waypoint_list.call(service_validate_waypoint_list))
  {
    std::string error_string = "Failed to call ValidateWaypointList";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  int error_number = service_validate_waypoint_list.response.output.trajectory_error_report.trajectory_error_elements.size();
  static const int MAX_ANGULAR_DURATION = 30;

  while (error_number >= 1 && angular_duration < MAX_ANGULAR_DURATION)
  {
    angular_duration++;
    trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration;

    service_validate_waypoint_list.request.input = trajectory;
    if (!service_client_validate_waypoint_list.call(service_validate_waypoint_list))
    {
      std::string error_string = "Failed to call ValidateWaypointList";
      ROS_ERROR("%s", error_string.c_str());
      return false;
    }
    error_number = service_validate_waypoint_list.response.output.trajectory_error_report.trajectory_error_elements.size();
  }

  if (angular_duration >= MAX_ANGULAR_DURATION)
  {
    // It should be possible to reach position within 30s
    // WaypointList is invalid (other error than angularWaypoint duration)
    std::string error_string = "WaypointList is invalid";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  service_execute_waypoints_trajectory.request.input = trajectory;

  // Send the angles
  if (service_client_execute_waypoints_trajectory.call(service_execute_waypoints_trajectory))
  {
    ROS_INFO("The joint angles were sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_send_gripper_command(ros::NodeHandle n, const std::string &robot_name, double value)
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
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool example_cartesian_waypoint(ros::NodeHandle n, const std::string &robot_name)
{
  
  ros::ServiceClient service_client_execute_waypoints_trajectory = n.serviceClient<kortex_driver::ExecuteWaypointTrajectory>("/" + robot_name + "/base/execute_waypoint_trajectory");
  ros::ServiceClient service_client_get_config = n.serviceClient<kortex_driver::GetProductConfiguration>("/" + robot_name + "/base/get_product_configuration");

  kortex_driver::ExecuteWaypointTrajectory service_execute_waypoints_trajectory;
  kortex_driver::GetProductConfiguration service_get_config;
  
  last_action_notification_event = 0;

  if (!service_client_get_config.call(service_get_config))
  {
    std::string error_string = "Failed to call GetProductConfiguration";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  auto product_config = service_get_config.response.output;

  if(product_config.model == kortex_driver::ModelId::MODEL_ID_L31) //If the robot is a GEN3-LITE use this trajectory.
  {
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.439,  0.194,  0.448, 90.6, -1.0, 150, 0));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.200,  0.150,  0.400, 90.6, -1.0, 150, 0));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.350,  0.050,  0.300, 90.6, -1.0, 150, 0));
  }
  else
  {
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.7,  0.0,   0.5,  90, 0, 90, 0));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.7,  0.0,   0.33, 90, 0, 90, 0.1));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.61, 0.22,  0.4,  90, 0, 90, 0.1));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.63, -0.22, 0.45, 90, 0, 90, 0.1));
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(FillCartesianWaypoint(0.65, 0.05,  0.45, 90, 0, 90, 0));
  }
  

  service_execute_waypoints_trajectory.request.input.duration = 0;
  service_execute_waypoints_trajectory.request.input.use_optimal_blending = 0;
  
  if (service_client_execute_waypoints_trajectory.call(service_execute_waypoints_trajectory))
  {
    ROS_INFO("The WaypointList command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "full_arm_movement_example_cpp");

  // For testing purpose
  ros::param::del("/kortex_examples_test_results/full_arm_movement_cpp");

  bool success = true;

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

  // Subscribe to the Action Topic
  ros::Subscriber sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);

  // We need to call this service to activate the Action Notification on the kortex_driver node.
  ros::ServiceClient service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;
  if (service_client_activate_notif.call(service_activate_notif))
  {
    ROS_INFO("Action notification activated!");
  }
  else 
  {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
    success = false;
  }

  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in fault
  success &= example_clear_faults(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Move the robot to the Home position with an Action
  success &= example_home_the_robot(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Example of gripper command
  // Let's close the gripper
  if (is_gripper_present)
  {
    success &= example_send_gripper_command(n, robot_name, 0.0);
  }
  //*******************************************************************************

  //*******************************************************************************
  // Set the reference frame to "Mixed"
  success &= example_set_cartesian_reference_frame(n, robot_name);

  // Example of cartesian pose
  // Let's make it move in Z
  success &= example_send_cartesian_pose(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Example of angular position
  // Let's send the arm to vertical position
  success &= example_send_joint_angles(n, robot_name, degrees_of_freedom);
  //*******************************************************************************

  //*******************************************************************************
  // Example of gripper command
  // Let's close the gripper
  if (is_gripper_present)
  {
    success &= example_send_gripper_command(n, robot_name, 0.5);
  }
  //*******************************************************************************

  //*******************************************************************************
  // Move the robot to the Home position again with an Action
  success &= example_home_the_robot(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Move the robot using Cartesian waypoint.
  success &= example_cartesian_waypoint(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Move the robot to the Home position one last time.
  success &= example_home_the_robot(n, robot_name);
  //*******************************************************************************

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/full_arm_movement_cpp", success);
  
  return success ? 0 : 1;
}