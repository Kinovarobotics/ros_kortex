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
#include "kortex_vision_config_driver/GetSensorSettings.h"
#include "kortex_vision_config_driver/Sensor.h"
#include "kortex_vision_config_driver/SetDeviceID.h"
#include "kortex_device_manager/DeviceTypes.h"
#include "kortex_device_manager/ReadAllDevices.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "VisionConfig_GetSensorSettings");

  ros::NodeHandle n;
  
  ros::ServiceClient service_GetSensorSettings = n.serviceClient<kortex_vision_config_driver::GetSensorSettings>("GetSensorSettings");
  ros::ServiceClient service_SetDeviceID = n.serviceClient<kortex_vision_config_driver::SetDeviceID>("SetDeviceID");
  ros::ServiceClient service_ReadAllDevices = n.serviceClient<kortex_device_manager::ReadAllDevices>("ReadAllDevices");
  
  kortex_vision_config_driver::GetSensorSettings srvGetSensorSettings;
  kortex_vision_config_driver::SetDeviceID srvSetDeviceID;
  kortex_device_manager::ReadAllDevices srvReadAllDevices;

  kortex_vision_config_driver::Sensor sensor_type;
  kortex_device_manager::DeviceTypes device_type;

  srvGetSensorSettings.request.input.sensor = sensor_type.SENSOR_COLOR;

  bool visionFound = false;
  uint32_t vision_id = 0;
  
  if (service_ReadAllDevices.call(srvReadAllDevices))
  {

    for(int i = 0; i < srvReadAllDevices.response.output.device_handle.size(); i++)
    {
      if(srvReadAllDevices.response.output.device_handle[i].device_type == device_type.VISION)
      {

        vision_id = srvReadAllDevices.response.output.device_handle[i].device_identifier;
        visionFound = true;
        srvSetDeviceID.request.device_id = vision_id;

        if (!service_SetDeviceID.call(srvSetDeviceID))
        {
          ROS_ERROR("Failed to call SetDeviceID");
        }
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call ReadAllDevices");
    return 1;
  }

  if(visionFound)
  {
    ROS_INFO("Found 1 vision module.\n");

    if (service_GetSensorSettings.call(srvGetSensorSettings))
    {
      ROS_INFO("sensor: %d", srvGetSensorSettings.response.output.sensor);
      ROS_INFO("resolution: %d", srvGetSensorSettings.response.output.resolution);
      ROS_INFO("frame_rate: %d", srvGetSensorSettings.response.output.frame_rate);
      ROS_INFO("bit_rate: %d", srvGetSensorSettings.response.output.bit_rate);
    }
    else
    {
      ROS_ERROR("Failed to call GetSensorSettings");
      return 1;
    }
  }
  else
  {
    ROS_INFO("Could not find any vision module on the target robot.");
  }

  return 0;
}