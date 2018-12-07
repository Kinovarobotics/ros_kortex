#include "ros/ros.h"
#include <kortex_actuator_driver/GetControlLoopParameters.h>
#include <kortex_actuator_driver/ControlLoopSelection.h>
#include "kortex_actuator_driver/SetDeviceID.h"
#include "kortex_device_manager/DeviceTypes.h"
#include "kortex_device_manager/ReadAllDevices.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Actuator_GetControlLoopParameters");

  ros::NodeHandle n;
  
  ros::ServiceClient service_GetControlLoopParameters = n.serviceClient<kortex_actuator_driver::GetControlLoopParameters>("GetControlLoopParameters");
  ros::ServiceClient service_SetDeviceID = n.serviceClient<kortex_actuator_driver::SetDeviceID>("SetDeviceID");
  ros::ServiceClient service_ReadAllDevices = n.serviceClient<kortex_device_manager::ReadAllDevices>("ReadAllDevices");
  
  kortex_actuator_driver::GetControlLoopParameters srvGetControlLoopParameters;
  kortex_actuator_driver::SetDeviceID srvSetDeviceID;
  kortex_device_manager::ReadAllDevices srvReadAllDevices;

  kortex_actuator_driver::ControlLoopSelection loopSelection;
  kortex_device_manager::DeviceTypes device_type;

  srvGetControlLoopParameters.request.input.loop_selection = loopSelection.MOTOR_VELOCITY;

  bool actuatorFound = false;
  uint32_t actuator_id = 0;

  if (service_ReadAllDevices.call(srvReadAllDevices))
  {

    for(int i = 0; i < srvReadAllDevices.response.output.device_handle.size(); i++)
    {
      if((srvReadAllDevices.response.output.device_handle[i].device_type == device_type.BIG_ACTUATOR) ||
         (srvReadAllDevices.response.output.device_handle[i].device_type == device_type.SMALL_ACTUATOR))
      {

        actuator_id = srvReadAllDevices.response.output.device_handle[i].device_identifier;
        actuatorFound = true;
        srvSetDeviceID.request.device_id = actuator_id;

        if (!service_SetDeviceID.call(srvSetDeviceID))
        {
          ROS_ERROR("Failed to call SetDeviceID");
        }

        break;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call ReadAllDevices");
    return 1;
  }

  if(actuatorFound)
  {
    ROS_INFO("Found 1 actuator.\n");

    if (service_GetControlLoopParameters.call(srvGetControlLoopParameters))
    {
      ROS_INFO("loop_selection: %d", srvGetControlLoopParameters.response.output.loop_selection);
      ROS_INFO("error_saturation: %f", srvGetControlLoopParameters.response.output.error_saturation);
      ROS_INFO("output_saturation: %f", srvGetControlLoopParameters.response.output.output_saturation);

      for(int i = 0; i < srvGetControlLoopParameters.response.output.kAz.size(); i++)
      {
        ROS_INFO("kAz[%d]: %f", i, srvGetControlLoopParameters.response.output.kAz[i]);
      }
      for(int i = 0; i < srvGetControlLoopParameters.response.output.kBz.size(); i++)
      {
        ROS_INFO("kBz[%d]: %f", i, srvGetControlLoopParameters.response.output.kBz[i]);
      }
    }
    else
    {
      ROS_ERROR("Failed to call GetControlLoopParameters");
      return 1;
    }
  }

  return 0;
}