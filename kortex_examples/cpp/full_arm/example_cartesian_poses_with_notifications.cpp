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
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/OnNotificationActionTopic.h>

bool Pose1Done = false;
bool Pose2Done = false;
bool Pose3Done = false;

//Callback that is called when the robot publish a ActionNotification
void notification_callback(const kortex_driver::ActionNotification::ConstPtr& notif)
{
  switch(notif->action_event)
  {
    //This event is published when an action has ended or was aborted
    case kortex_driver::ActionEvent::ACTION_END:
    {
      ROS_INFO("Action has ended!");
      if(notif->handle.identifier == 1001)
      {
        Pose1Done = true;
      }

      if(notif->handle.identifier == 1002)
      {
        Pose2Done = true;
      }

      if(notif->handle.identifier == 1003)
      {
        Pose3Done = true;
      }
      break;
    }
    case kortex_driver::ActionEvent::ACTION_ABORT:
    {
      ROS_ERROR("Action was aborted!");
      if(notif->handle.identifier == 1001)
      {
        Pose1Done = true;
      }

      if(notif->handle.identifier == 1002)
      {
        Pose2Done = true;
      }

      if(notif->handle.identifier == 1003)
      {
        Pose3Done = true;
      }
      break;
    }
    default:
      break;
  }
  
}

void example_cartesian_action(ros::NodeHandle n, std::string robot_name)
{
  ros::ServiceClient service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;

  //We need to call this service to activate the Action Notification on the kortex_driver node.
  if (service_client_activate_notif.call(service_activate_notif))
  {
    ROS_INFO("Action notification activated!");
  }
  else 
  {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  ros::Duration(1.00).sleep();

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.1f;
  my_cartesian_speed.orientation = 15.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.374f;
  my_constrained_pose.target_pose.y = 0.081f;
  my_constrained_pose.target_pose.z = 0.450f;
  my_constrained_pose.target_pose.theta_x = -57.6f;
  my_constrained_pose.target_pose.theta_y = 91.1f;
  my_constrained_pose.target_pose.theta_z = 2.3f;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // Waiting for the pose 1 to end
  while(!Pose1Done)
  {
    ros::spinOnce();
  }

  // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1002;
  service_execute_action.request.input.name = "pose2";

  my_constrained_pose.target_pose.z = 0.3f;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);

  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  //Waiting for the pose 2 to end.
  while(!Pose2Done)
  {
    ros::spinOnce();
  }

  // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1003;
  service_execute_action.request.input.name = "pose3";
  
  my_constrained_pose.target_pose.z = 0.20f;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);

  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 3 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }

  // Waiting for the pose 3 to end
  while(!Pose3Done)
  {
    ros::spinOnce();
  }
}

// This function sets the reference frame to the robot's base
void example_set_cartesian_reference_frame(ros::NodeHandle n, std::string robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    throw new std::runtime_error(error_string);
  }
  else
  {
    ROS_INFO("Set reference to base frame.");
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "full_arm_cartesian_action_cpp");

  //*******************************************************************************
  // ROS Parameters
  ros::NodeHandle n;
  std::string robot_name = "my_gen3";

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

  ros::Subscriber sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);
  example_set_cartesian_reference_frame(n, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  example_cartesian_action(n, robot_name);

  return 0;
}