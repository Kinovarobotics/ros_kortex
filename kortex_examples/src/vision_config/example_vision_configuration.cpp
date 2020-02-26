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

#include "ros/ros.h"
#include <iostream>
#include <kortex_driver/VisionNotification.h>
#include <kortex_driver/OnNotificationVisionTopic.h>
#include <kortex_driver/GetIntrinsicParameters.h>
#include <kortex_driver/Sensor.h>
#include <kortex_driver/Resolution.h>
#include <kortex_driver/FrameRate.h>
#include <kortex_driver/BitRate.h>
#include <kortex_driver/GetExtrinsicParameters.h>
#include <kortex_driver/GetSensorSettings.h>
#include <kortex_driver/SetSensorSettings.h>
#include <kortex_driver/GetOptionValue.h>
#include <kortex_driver/SetOptionValue.h>
#include <kortex_driver/Option.h>

std::string sensor_type_enum_to_string(uint32_t enum_value)
{
  std::string s = "";
  switch (enum_value)
  {
    case kortex_driver::Sensor::SENSOR_COLOR:
      s = "Color";
      break;

    case kortex_driver::Sensor::SENSOR_DEPTH:
      s = "Depth";
      break;

    default:
    case kortex_driver::Sensor::SENSOR_UNSPECIFIED:
      s = "Unspecified";
      break;
  }
  return s;
}

std::string resolution_enum_to_string(uint32_t enum_value)
{
  std::string s = "";
  switch (enum_value)
  {
    case kortex_driver::Resolution::RESOLUTION_320x240:
      s = "320 x 240";
      break;
    case kortex_driver::Resolution::RESOLUTION_424x240:
      s = "424 x 240";
      break;
    case kortex_driver::Resolution::RESOLUTION_480x270:
      s = "480 x 270";
      break;
    case kortex_driver::Resolution::RESOLUTION_640x480:
      s = "640 x 480";
      break;
    case kortex_driver::Resolution::RESOLUTION_1280x720:
      s = "1280 x 720";
      break;
    case kortex_driver::Resolution::RESOLUTION_1920x1080:
      s = "1920 x 1080";
      break;
    default:
    case kortex_driver::Resolution::RESOLUTION_UNSPECIFIED:
      s = "Unspecified";
      break;
  }
  return s;
}

std::string framerate_enum_to_string(uint32_t enum_value)
{
  std::string s = "";
  switch (enum_value)
  {
    case kortex_driver::FrameRate::FRAMERATE_6_FPS:
      s = "6 FPS";
      break;
    case kortex_driver::FrameRate::FRAMERATE_15_FPS:
      s = "15 FPS";
      break;
    case kortex_driver::FrameRate::FRAMERATE_30_FPS:
      s = "30 FPS";
      break;
    default:
    case kortex_driver::FrameRate::FRAMERATE_UNSPECIFIED:
      s = "Unspecified";
      break;
  }
  return s;
}

std::string bitrate_enum_to_string(uint32_t enum_value)
{
  std::string s = "";
  switch (enum_value)
  {
    case kortex_driver::BitRate::BITRATE_10_MBPS:
      s = "10 MBPS";
      break;
    case kortex_driver::BitRate::BITRATE_15_MBPS:
      s = "15 MBPS";
      break;
    case kortex_driver::BitRate::BITRATE_20_MBPS:
      s = "20 MBPS";
      break;
    case kortex_driver::BitRate::BITRATE_25_MBPS:
      s = "25 MBPS";
      break;
    default:
    case kortex_driver::BitRate::BITRATE_UNSPECIFIED:
      s = "Unspecified";
      break;
  }
  return s;
}

bool example_get_intrinsic_parameters(ros::NodeHandle n, const std::string& robot_name)
{
  ros::ServiceClient service_client_get_intrinsic_parameters = n.serviceClient<kortex_driver::GetIntrinsicParameters>("/" + robot_name + "/vision_config/get_intrinsic_parameters");
  kortex_driver::GetIntrinsicParameters service_get_intrinsic_parameters;

  // Get the parameters from the color sensor
  kortex_driver::GetIntrinsicParametersRequest req;
  req.input.sensor = kortex_driver::Sensor::SENSOR_COLOR; // Change to SENSOR_DEPTH for the depth sensor
  service_get_intrinsic_parameters.request = req;

  if (!service_client_get_intrinsic_parameters.call(service_get_intrinsic_parameters))
  {
    std::string error_string = "Failed to call GetIntrinsicParameters";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  auto output = service_get_intrinsic_parameters.response.output;
  // The message description can be seen at msg/generated/vision_config/IntrinsicParameters.msg
  std::ostringstream oss;
  oss << std::endl << "---------------------------------" << std::endl
  << "Intrinsic parameters are : " << std::endl
  << "Focal length in x is : " << output.focal_length_x << std::endl
  << "Focal length in y is : " << output.focal_length_y << std::endl
  << "Principal point in x is : " << output.principal_point_x << std::endl
  << "Principal point in y is : " << output.principal_point_y << std::endl
  << "Distortion coefficients are : [" << 
  "k1 = " << output.distortion_coeffs.k1 << "; " <<
  "k2 = " << output.distortion_coeffs.k2 << "; " <<
  "k3 = " << output.distortion_coeffs.k3 << "; " <<
  "p1 = " << output.distortion_coeffs.p1 << "; " <<
  "p2 = " << output.distortion_coeffs.p2 << "]" << std::endl;
  
  // The Sensor enum can be seen at msg/generated/vision_config/Sensor.msg
  oss << "Sensor type is : " << sensor_type_enum_to_string(output.sensor) << std::endl;

  // The Resolution enum can be seen at msg/generated/vision_config/Resolution.msg
  oss << "Resolution is : " << resolution_enum_to_string(output.resolution) << std::endl 
  << "---------------------------------";

  ROS_INFO("%s", oss.str().c_str());

  return true;
}

bool example_get_extrinsic_parameters(ros::NodeHandle n, const std::string& robot_name)
{
  ros::ServiceClient service_client_get_extrinsic_parameters = n.serviceClient<kortex_driver::GetExtrinsicParameters>("/" + robot_name + "/vision_config/get_extrinsic_parameters");
  kortex_driver::GetExtrinsicParameters service_get_extrinsic_parameters;

  kortex_driver::GetExtrinsicParametersRequest req;

  if (!service_client_get_extrinsic_parameters.call(service_get_extrinsic_parameters))
  {
    std::string error_string = "Failed to call GetExtrinsicParameters";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  auto output = service_get_extrinsic_parameters.response.output;
  // The message description can be seen at msg/generated/vision_config/ExtrinsicParameters.msg
  std::ostringstream oss;
  oss << std::endl << "---------------------------------" << std::endl
  << "Extrinsic parameters are : " << std::endl
  << "Rotation parameters matrix is : " << std::endl
  << "|  " << output.rotation.row1.column1 << "  ;  " << output.rotation.row1.column2 << "  ;  " << output.rotation.row1.column3 << "  |" << std::endl
  << "|  " << output.rotation.row2.column1 << "  ;  " << output.rotation.row2.column2 << "  ;  " << output.rotation.row2.column3 << "  |" << std::endl
  << "|  " << output.rotation.row3.column1 << "  ;  " << output.rotation.row3.column2 << "  ;  " << output.rotation.row3.column3 << "  |" << std::endl;

  oss << "Translation parameters are : " <<
  "[ x = " << output.translation.t_x <<
  " ; y = " << output.translation.t_y << 
  " ; z = " << output.translation.t_z << " ]"
  << std::endl << "---------------------------------" << std::endl;

  ROS_INFO("%s", oss.str().c_str());

  return true;
}

bool example_get_sensor_settings(ros::NodeHandle n, const std::string& robot_name)
{
  ros::ServiceClient service_client_get_sensor_settings = n.serviceClient<kortex_driver::GetSensorSettings>("/" + robot_name + "/vision_config/get_sensor_settings");
  kortex_driver::GetSensorSettings service_get_sensor_settings;

  kortex_driver::GetSensorSettingsRequest req;

  // Get settings for the color sensor
  req.input.sensor = kortex_driver::Sensor::SENSOR_COLOR;
  service_get_sensor_settings.request = req;

  if (!service_client_get_sensor_settings.call(service_get_sensor_settings))
  {
    std::string error_string = "Failed to call GetSensorSettings";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  auto output = service_get_sensor_settings.response.output;

  std::ostringstream oss;
  oss << std::endl << 
  "---------------------------------" << std::endl <<
  "Get sensor settings : " << std::endl <<
  "Bit rate : " << bitrate_enum_to_string(output.bit_rate) << std::endl <<
  "Frame rate : " << framerate_enum_to_string(output.frame_rate) << std::endl << 
  "Resolution : " << resolution_enum_to_string(output.resolution) << std::endl << 
  "---------------------------------";

  ROS_INFO("%s", oss.str().c_str());

  return true;
}

bool example_change_the_resolution(ros::NodeHandle n, const std::string& robot_name)
{
  ros::ServiceClient service_client_set_sensor_settings = n.serviceClient<kortex_driver::SetSensorSettings>("/" + robot_name + "/vision_config/set_sensor_settings");
  kortex_driver::SetSensorSettings service_set_sensor_settings;

  kortex_driver::SetSensorSettingsRequest req;

  ROS_INFO("Changing the resolution...");

  // Set the resolution to be 640 x 480 on the color sensor
  // You have to specify all parameters else the call will return an INVALID_PARAM error
  req.input.sensor = kortex_driver::Sensor::SENSOR_COLOR;
  req.input.resolution = kortex_driver::Resolution::RESOLUTION_1280x720;
  req.input.bit_rate = kortex_driver::BitRate::BITRATE_10_MBPS;
  req.input.frame_rate = kortex_driver::FrameRate::FRAMERATE_30_FPS;

  service_set_sensor_settings.request = req;

  if (!service_client_set_sensor_settings.call(service_set_sensor_settings))
  {
    std::string error_string = "Failed to call SetSensorSettings";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  ROS_INFO("Resolution changed successfully.");

  return true;
}

bool example_get_sensor_option_value(ros::NodeHandle n, const std::string& robot_name)
{
  ros::ServiceClient service_client_get_sensor_option_value = n.serviceClient<kortex_driver::GetOptionValue>("/" + robot_name + "/vision_config/get_option_value");
  kortex_driver::GetOptionValue service_get_sensor_option_value;

  kortex_driver::GetOptionValueRequest req;

  // The only supported options for now are:
  // For Color sensor : 
  // OPTION_BRIGHTNESS, OPTION_CONTRAST, OPTION_SATURATION
  // For Depth sensor : 
  // OPTION_EXPOSURE, OPTION_GAIN, OPTION_ENABLE_AUTO_EXPOSURE, OPTION_VISUAL_PRESET, OPTION_FRAMES_QUEUE_SIZE, 
  // OPTION_ERROR_POLLING_ENABLE, OPTION_OUTPUT_TRIGGER_ENABLED, OPTION_DEPTH_UNITS, 
  // OPTION_STEREO_BASELINE (read-only) 

  // Trying to call an unsupported option in this service (or in the SetOptionValue service) will generate an error 
  // Get the actual value the color sensor's contrast 
  req.input.sensor = kortex_driver::Sensor::SENSOR_COLOR;
  req.input.option = kortex_driver::Option::OPTION_CONTRAST;

  service_get_sensor_option_value.request = req;

  if (!service_client_get_sensor_option_value.call(service_get_sensor_option_value))
  {
    std::string error_string = "Failed to call GetOptionValue";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  auto output = service_get_sensor_option_value.response.output;
  std::ostringstream oss;
  oss << std::endl << "---------------------------------" << std::endl
  << "Get Option value : " << std::endl
  << "Option value is : " << std::endl
  << "For sensor : " << output.sensor << std::endl
  << "For option : " << output.option << std::endl
  << "The value is : " << output.value << std::endl
  << "---------------------------------" << std::endl;

  ROS_INFO("%s", oss.str().c_str());

  return true;
}

int main(int argc, char **argv)
{
  // Init the node and get the namespace parameter
  ros::init(argc, argv, "vision_configuration_example_cpp");

  // For testing purpose
  ros::param::del("/kortex_examples_test_results/vision_configuration_cpp");

  bool success = true;

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

  //-------------------------------------------------------------
  // Get the intrinsic parameters for a given sensor
  success &= example_get_intrinsic_parameters(n, robot_name);

  //-------------------------------------------------------------
  // Get the extrinsic parameters for a given sensor
  success &= example_get_extrinsic_parameters(n, robot_name);

  //-------------------------------------------------------------
  // Get the sensor settings for a given sensor
  success &= example_get_sensor_settings(n, robot_name);

  //-------------------------------------------------------------
  // Set the extrinsic parameters for a given sensor
  success &= example_change_the_resolution(n, robot_name);

  //-------------------------------------------------------------
  // Get an option value
  success &= example_get_sensor_option_value(n, robot_name);

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/vision_configuration_cpp", success);
  
  return success ? 0 : 1;
}