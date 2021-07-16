/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2021 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

#include <thread>
#include <atomic>
#include <math.h>

#include <ros/ros.h>
#include <angles/angles.h>

#include "actionlib/client/simple_action_client.h"
#include "kortex_driver/Base_ClearFaults.h"
#include "kortex_driver/BaseCyclic_Feedback.h"
#include "kortex_driver/ReadAction.h"
#include "kortex_driver/ExecuteAction.h"
#include "kortex_driver/ActionNotification.h"
#include "kortex_driver/ActionEvent.h"
#include "kortex_driver/OnNotificationActionTopic.h"
#include <kortex_driver/CartesianReferenceFrame.h>
#include "kortex_driver/FollowCartesianTrajectoryAction.h"
#include "kortex_driver/FollowCartesianTrajectoryActionFeedback.h"
#include "kortex_driver/FollowCartesianTrajectoryActionGoal.h"
#include "kortex_driver/FollowCartesianTrajectoryActionResult.h"
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

kortex_driver::CartesianWaypoint FillCartesianWaypoint(float new_x, float new_y, float new_z, float new_theta_x, float new_theta_y, float new_theta_z, float blending_radius)
{
  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = new_x;
  cartesianWaypoint.pose.y = new_y;
  cartesianWaypoint.pose.z = new_z;
  cartesianWaypoint.pose.theta_x = new_theta_x;
  cartesianWaypoint.pose.theta_y = new_theta_y;
  cartesianWaypoint.pose.theta_z = new_theta_z;
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = blending_radius;

  return cartesianWaypoint;
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

bool example_cartesian_waypoint_action(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_get_config = n.serviceClient<kortex_driver::GetProductConfiguration>("/" + robot_name + "/base/get_product_configuration");
  kortex_driver::GetProductConfiguration service_get_config;
  
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  ROS_INFO("Waiting for the server.");
  
  // wait for the action server to start
  ac.waitForServer();

  ROS_INFO("Server responded.");

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  if (!service_client_get_config.call(service_get_config))
  {
    std::string error_string = "Failed to call GetProductConfiguration";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  auto product_config = service_get_config.response.output;

  if(product_config.model == kortex_driver::ModelId::MODEL_ID_L31) //If the robot is a GEN3-LITE use this trajectory.
  {
    goal.trajectory.push_back(FillCartesianWaypoint(0.439,  0.194,  0.448, angles::from_degrees(90.6), angles::from_degrees(-1.0), angles::from_degrees(150), 0));
    goal.trajectory.push_back(FillCartesianWaypoint(0.200,  0.150,  0.400, angles::from_degrees(90.6), angles::from_degrees(-1.0), angles::from_degrees(150), 0));
    goal.trajectory.push_back(FillCartesianWaypoint(0.350,  0.050,  0.300, angles::from_degrees(90.6), angles::from_degrees(-1.0), angles::from_degrees(150), 0));
  }
  else
  {
    goal.trajectory.push_back(FillCartesianWaypoint(0.7,  0.0,   0.5,  angles::from_degrees(90), 0, angles::from_degrees(90), 0));
    goal.trajectory.push_back(FillCartesianWaypoint(0.7,  0.0,   0.33, angles::from_degrees(90), 0, angles::from_degrees(90), 0.1));
    goal.trajectory.push_back(FillCartesianWaypoint(0.7,  0.48,  0.33, angles::from_degrees(90), 0, angles::from_degrees(90), 0.1));
    goal.trajectory.push_back(FillCartesianWaypoint(0.61, 0.22,  0.4,  angles::from_degrees(90), 0, angles::from_degrees(90), 0.1));
    goal.trajectory.push_back(FillCartesianWaypoint(0.7,  0.48,  0.33, angles::from_degrees(90), 0, angles::from_degrees(90), 0.1));
    goal.trajectory.push_back(FillCartesianWaypoint(0.63, -0.22, 0.45, angles::from_degrees(90), 0, angles::from_degrees(90), 0.1));
    goal.trajectory.push_back(FillCartesianWaypoint(0.65, 0.05,  0.45, angles::from_degrees(90), 0, angles::from_degrees(90), 0));
  }

  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }

  //exit
  return completed;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_action_cpp");

  // For testing purpose
  ros::param::del("/kortex_examples_test_results/waypoint_action_cpp");

  bool success = true;

  //*******************************************************************************
  // ROS Parameters
  ros::NodeHandle n;

  //Here, put the name of your robot
  std::string robot_name = "my_gen3";

  //Here you specify your robot's degree of freedom
  int degrees_of_freedom = 7;

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
  // Move the robot using Cartesian waypoint with the action server.
  success &= example_cartesian_waypoint_action(n, robot_name);
  //*******************************************************************************

  //*******************************************************************************
  // Move the robot to the Home position one last time.
  success &= example_home_the_robot(n, robot_name);
  //*******************************************************************************

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/waypoint_action_cpp", success);
  
  return success ? 0 : 1;
}