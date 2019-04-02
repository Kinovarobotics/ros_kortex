#include "ros/ros.h"
#include <kortex_driver/CreateUserProfile.h>
#include <kortex_driver/DeleteUserProfile.h>
#include <kortex_driver/ConfigurationChangeNotification.h>
#include <kortex_driver/OnNotificationConfigurationChangeTopic.h>
#include <kortex_driver/ConfigurationNotificationEvent.h>

void ProfileCallback(const kortex_driver::ConfigurationChangeNotification::ConstPtr& notif)
{
  ROS_INFO("Received Notification");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tests");

  ros::NodeHandle n;
                                             
  ros::Subscriber sub = n.subscribe("ConfigurationChangeTopic", 1000, ProfileCallback);

  ros::ServiceClient service_CreateUserProfile = n.serviceClient<kortex_driver::CreateUserProfile>("CreateUserProfile");
  ros::ServiceClient service_DeleteUserProfile = n.serviceClient<kortex_driver::DeleteUserProfile>("DeleteUserProfile");
  ros::ServiceClient service_OnNotificationConfigurationChangeTopic = n.serviceClient<kortex_driver::OnNotificationConfigurationChangeTopic>("OnNotificationConfigurationChangeTopic");
  
  kortex_driver::CreateUserProfile srvCreateUserProfile;
  kortex_driver::DeleteUserProfile srvDeleteUserProfile;
  kortex_driver::OnNotificationConfigurationChangeTopic srvOnNotificationConfigurationChangeTopic;
  
  srvCreateUserProfile.request.input.user_profile.username = "jcash";
  srvCreateUserProfile.request.input.user_profile.firstname = "Johnny";
  srvCreateUserProfile.request.input.user_profile.lastname = "Cash";
  srvCreateUserProfile.request.input.user_profile.application_data = "Custom Application Stuff";
  srvCreateUserProfile.request.input.password = "pwd";

  if (service_OnNotificationConfigurationChangeTopic.call(srvOnNotificationConfigurationChangeTopic))
  {
    ROS_INFO("topic registered");
  }
  else
  {
    ROS_ERROR("registration error");
  }

  ros::Duration(2).sleep();

  if (service_CreateUserProfile.call(srvCreateUserProfile))
  {
    ROS_INFO("Created user profile with handle: %d", srvCreateUserProfile.response.output.identifier);
    srvDeleteUserProfile.request.input.identifier = srvCreateUserProfile.response.output.identifier;

    if (service_DeleteUserProfile.call(srvDeleteUserProfile))
    {
      ROS_INFO("Deleted successfully");
    }
    else
    {
      ROS_ERROR("Failed to call DeleteUserProfile");
      return 1;
    }

  }
  else
  {
    ROS_ERROR("Failed to call CreateUserProfile");
    return 1;
  }

  ros::Duration(2).sleep();

  ros::spinOnce();

  return 0;
}