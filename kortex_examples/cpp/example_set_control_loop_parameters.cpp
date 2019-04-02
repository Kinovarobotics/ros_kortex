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
#include "kortex_actuator_driver/SetControlLoopParameters.h"
#include "kortex_actuator_driver/ControlLoopSelection.h"
#include "kortex_actuator_driver/SetDeviceID.h"
#include "kortex_device_manager/DeviceTypes.h"
#include "kortex_device_manager/ReadAllDevices.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Example_Actuator_SetControlLoopParameters");

  ros::NodeHandle n;
  
  ros::ServiceClient service_SetControlLoopParameters = n.serviceClient<kortex_actuator_driver::SetControlLoopParameters>("SetControlLoopParameters");
  ros::ServiceClient service_SetDeviceID = n.serviceClient<kortex_actuator_driver::SetDeviceID>("SetDeviceID");
  ros::ServiceClient service_ReadAllDevices = n.serviceClient<kortex_device_manager::ReadAllDevices>("ReadAllDevices");

  
  kortex_actuator_driver::SetControlLoopParameters srvSetControlLoopParameters;
  kortex_actuator_driver::ControlLoopSelection loopSelection;
  kortex_actuator_driver::SetDeviceID srvSetDeviceID;
  kortex_device_manager::ReadAllDevices srvReadAllDevices;

  srvSetControlLoopParameters.request.input.loop_selection = loopSelection.MOTOR_VELOCITY;
  kortex_device_manager::DeviceTypes device_type;

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
    if (service_SetControlLoopParameters.call(srvSetControlLoopParameters))
    {
      ROS_INFO("ControlLoopParameters sent with success");
    }
    else
    {
      ROS_ERROR("Failed to call SetControlLoopParameters");
      return 1;
    }
  }

  return 0;
}