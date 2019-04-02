/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2018 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

#include "ros/ros.h"
#include <kortex_driver/PlayCartesianTrajectory.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/RefreshFeedback.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Sequence");

  ros::NodeHandle n;
  
  ros::ServiceClient client_PlayCartesianTrajectory = n.serviceClient<kortex_driver::PlayCartesianTrajectory>("PlayCartesianTrajectory");
  ros::ServiceClient client_RefreshFeedback = n.serviceClient<kortex_driver::RefreshFeedback>("RefreshFeedback");
  
  kortex_driver::PlayCartesianTrajectory srvPlayCartesianTrajectory;
  kortex_driver::RefreshFeedback srvRefreshFeedback;

  float current_x = 0.0f;
  float current_y = 0.0f;
  float current_z = 0.0f;

  float current_theta_x = 0.0f;
  float current_theta_y = 0.0f;
  float current_theta_z = 0.0f;

  if (client_RefreshFeedback.call(srvRefreshFeedback))
  {
    current_x = srvRefreshFeedback.response.output.base.tool_pose_x;
    current_y = srvRefreshFeedback.response.output.base.tool_pose_y;
    current_z = srvRefreshFeedback.response.output.base.tool_pose_z;

    current_theta_x = srvRefreshFeedback.response.output.base.tool_pose_theta_x;
    current_theta_y = srvRefreshFeedback.response.output.base.tool_pose_theta_y;
    current_theta_z = srvRefreshFeedback.response.output.base.tool_pose_theta_z;

    ROS_INFO("Getting cyclic data from the robot - x= %f   y= %f   z= %f      theta x = %f   theta y = %f   theta z = %f", 
              current_x, current_y, current_z, current_theta_x, current_theta_y, current_theta_z);
  }
  else
  {
    ROS_ERROR("Failed to retrieve the cyclic data.");
    return 1;
  }

  //Creating our next target (a Cartesian pose)
  srvPlayCartesianTrajectory.request.input.target_pose.x = current_x;
  srvPlayCartesianTrajectory.request.input.target_pose.y = current_y;
  srvPlayCartesianTrajectory.request.input.target_pose.z = current_z + 0.1;

  srvPlayCartesianTrajectory.request.input.target_pose.theta_x = current_theta_x;
  srvPlayCartesianTrajectory.request.input.target_pose.theta_y = current_theta_y;
  srvPlayCartesianTrajectory.request.input.target_pose.theta_z = current_theta_z;

  kortex_driver::CartesianSpeed poseSpeed;
  poseSpeed.translation = 0.1;
  poseSpeed.orientation = 15;

  srvPlayCartesianTrajectory.request.input.constraint.oneof_type.speed.push_back(poseSpeed);

  ROS_INFO("CALL PlayCartesianTrajectory");
  if (client_PlayCartesianTrajectory.call(srvPlayCartesianTrajectory))
  {
    ROS_INFO("pose sent");
  }
  else
  {
    ROS_ERROR("Failed to call PlayCartesianTrajectory");
    return 1;
  }

  return 0;
}