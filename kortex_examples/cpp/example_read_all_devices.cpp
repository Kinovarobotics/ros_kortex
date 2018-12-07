#include "ros/ros.h"
#include "kortex_device_manager/ReadAllDevices.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Example_DeviceManager_ReadAllDevices");

  ros::NodeHandle n;
  
  ros::ServiceClient service_ReadAllDevices = n.serviceClient<kortex_device_manager::ReadAllDevices>("ReadAllDevices");
  
  kortex_device_manager::ReadAllDevices srvReadAllDevices;

  ROS_INFO("CALL ReadAllDevices");

  if (service_ReadAllDevices.call(srvReadAllDevices))
  {
    for(int i = 0; i < srvReadAllDevices.response.output.device_handle.size(); i++)
    {
      ROS_INFO("device_type: %d", srvReadAllDevices.response.output.device_handle[i].device_type);
      ROS_INFO("device_identifier: %d", srvReadAllDevices.response.output.device_handle[i].device_identifier);
      ROS_INFO("order: %d", srvReadAllDevices.response.output.device_handle[i].order);
    }
  }
  else
  {
    ROS_ERROR("Failed to call ReadAllDevices");
    return 1;
  }

  return 0;
}