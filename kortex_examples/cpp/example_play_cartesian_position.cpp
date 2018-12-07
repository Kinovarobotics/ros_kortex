#include "ros/ros.h"
#include <kortex_driver/PlayCartesianTrajectoryPosition.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/RefreshFeedback.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_play_cartesian_position");

  ros::NodeHandle n;
  
  ros::ServiceClient client_PlayCartesianTrajectoryPosition = n.serviceClient<kortex_driver::PlayCartesianTrajectoryPosition>("PlayCartesianTrajectoryPosition");
  ros::ServiceClient client_RefreshFeedback = n.serviceClient<kortex_driver::RefreshFeedback>("RefreshFeedback");
  
  kortex_driver::PlayCartesianTrajectoryPosition srvPlayCartesianTrajectoryPosition;
  kortex_driver::RefreshFeedback srvRefreshFeedback;

  float current_x = 0.0f;
  float current_y = 0.0f;
  float current_z = 0.0f;

  if (client_RefreshFeedback.call(srvRefreshFeedback))
  {
    current_x = srvRefreshFeedback.response.output.base.tool_pose_x;
    current_y = srvRefreshFeedback.response.output.base.tool_pose_y;
    current_z = srvRefreshFeedback.response.output.base.tool_pose_z;
    ROS_INFO("Getting cyclic data from the robot - x= %f   y= %f   z= %f", current_x, current_y, current_z);
  }
  else
  {
    ROS_ERROR("Failed to retrieve the cyclic data.");
    return 1;
  }

  

  //Creating our next target (a Cartesian pose)
  srvPlayCartesianTrajectoryPosition.request.input.target_position.x = current_x;
  srvPlayCartesianTrajectoryPosition.request.input.target_position.y = current_y;
  srvPlayCartesianTrajectoryPosition.request.input.target_position.z = current_z + 0.1;

  kortex_driver::CartesianSpeed poseSpeed;
  poseSpeed.translation = 0.1;

  srvPlayCartesianTrajectoryPosition.request.input.constraint.oneof_type.speed.push_back(poseSpeed);

  ROS_INFO("CALL PlayCartesianTrajectory");
  if (client_PlayCartesianTrajectoryPosition.call(srvPlayCartesianTrajectoryPosition))
  {
    ROS_INFO("pose sent");
  }
  else
  {
    ROS_ERROR("Failed to call PlayCartesianTrajectoryPosition");
    return 1;
  }

  return 0;
}