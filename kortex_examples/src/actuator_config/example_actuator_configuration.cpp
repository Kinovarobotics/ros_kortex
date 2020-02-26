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
#include <std_msgs/Empty.h>
#include <iostream>
#include <kortex_driver/ReadAllDevices.h>
#include <kortex_driver/DeviceHandle.h>
#include <kortex_driver/DeviceTypes.h>
#include <kortex_driver/SetDeviceID.h>
#include <kortex_driver/GetControlLoopParameters.h>
#include <kortex_driver/SetControlLoopParameters.h>
#include <kortex_driver/ActuatorConfig_ClearFaults.h>
#include <kortex_driver/ControlLoopSelection.h>

typedef kortex_driver::GetControlLoopParameters::Response::_output_type ControlLoopParameters;

std::string control_loop_to_string(uint32_t int_control_loop)
{
  switch (int_control_loop)
  {
    case kortex_driver::ControlLoopSelection::RESERVED:
      return "RESERVED";
    case kortex_driver::ControlLoopSelection::JOINT_POSITION:
      return "JOINT POSITION";
    case kortex_driver::ControlLoopSelection::MOTOR_POSITION:
      return "MOTOR POSITION";
    case kortex_driver::ControlLoopSelection::JOINT_VELOCITY:
      return "JOINT VELOCITY";
    case kortex_driver::ControlLoopSelection::MOTOR_VELOCITY:
      return "MOTOR VELOCITY";
    case kortex_driver::ControlLoopSelection::JOINT_TORQUE:
      return "JOINT TORQUE";
    case kortex_driver::ControlLoopSelection::MOTOR_CURRENT:
      return "MOTOR CURRENT";
    default:
      return "UNKNOWN CONTROL LOOP TYPE";
  }
}

bool example_find_actuators_and_set_device_id(ros::NodeHandle& n, const std::string& robot_name, uint32_t device_id)
{
  ros::ServiceClient service_client_read_all_devices = n.serviceClient<kortex_driver::ReadAllDevices>("/" + robot_name + "/device_manager/read_all_devices");
  kortex_driver::ReadAllDevices service_read_all_devices;
  kortex_driver::DeviceTypes device_type;
  std::vector<uint32_t> device_id_vector;

  std::ostringstream oss;
  oss << std::endl;

  if (service_client_read_all_devices.call(service_read_all_devices))
  {
    auto output = service_read_all_devices.response.output;
    // Cycle through all found devices to find actuators
    for(int i = 0; i < output.device_handle.size(); i++)
    {

      if((output.device_handle[i].device_type == device_type.BIG_ACTUATOR) ||
          (output.device_handle[i].device_type == device_type.MEDIUM_ACTUATOR) ||
          (output.device_handle[i].device_type == device_type.SMALL_ACTUATOR))
      {
        // Add the device_id to the vector if we found an actuator
        device_id_vector.push_back(output.device_handle[i].device_identifier);
        oss << "Found an actuator with device id " << output.device_handle[i].device_identifier << std::endl;
      }
    }
  }
  else
  {
    std::string error_string = "Failed to call ReadAllDevices"; 
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  oss << "----------------------------";
  ROS_INFO("%s", oss.str().c_str());
  oss.clear();

  // Check if the specified device_id is in the vector
  if (std::find(device_id_vector.begin(), device_id_vector.end(), device_id) != device_id_vector.end())
  {
    ROS_INFO("Device id %u is a valid actuator device id.", device_id);
  }
  else
  {
    std::string error_string = "Device id " + std::to_string(device_id) + " does not correspond to an actuator's device id."; 
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // We need to set the device ID of the actuator we want to configure
  ros::ServiceClient service_client_set_device_id = n.serviceClient<kortex_driver::SetDeviceID>("/" + robot_name + "/actuator_config/set_device_id");
  kortex_driver::SetDeviceID service_set_device_id;
  service_set_device_id.request.device_id = device_id;
  if (service_client_set_device_id.call(service_set_device_id))
  {
    ROS_INFO("Device ID was properly set.");
  }
  else
  {
    std::string error_string = "Failed to call SetDeviceID"; 
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return true;

}

bool example_clear_actuator_faults(ros::NodeHandle& n, const std::string& robot_name)
{
  ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::ActuatorConfig_ClearFaults>("/" + robot_name + "/actuator_config/clear_faults");
  kortex_driver::ActuatorConfig_ClearFaults service_clear_faults;
  if (service_client_clear_faults.call(service_clear_faults))
  {
    ROS_INFO("Faults were cleared properly.");
    return true;
  }
  else
  {
    std::string error_string = "Failed to call ActuatorConfig_ClearFaults"; 
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
}

bool example_get_control_loop_parameters(ros::NodeHandle& n, const std::string& robot_name, ControlLoopParameters& output)
{
  // Get the actuator control loop parameters
  ros::ServiceClient service_client_get_control_loop_parameters = n.serviceClient<kortex_driver::GetControlLoopParameters>("/" + robot_name + "/actuator_config/get_control_loop_parameters");
  kortex_driver::GetControlLoopParameters service_get_control_loop_parameters;
  std::ostringstream oss;
  if (service_client_get_control_loop_parameters.call(service_get_control_loop_parameters))
  {
    // The msg file can be found at kortex_driver/msg/generated/actuator_config/ControlLoopParameters.msg
    kortex_driver::GetControlLoopParametersResponse::_output_type initial_parameters = service_get_control_loop_parameters.response.output;
    oss << std::endl << "Control Loop Parameters : " << std::endl;
    oss << "Loop selection : " << control_loop_to_string(initial_parameters.loop_selection) << std::endl;
    oss << "Error saturation : " << initial_parameters.error_saturation << std::endl;
    oss << "Output saturation : " << initial_parameters.output_saturation << std::endl;
    oss << "kAz : [";
    for (auto element : initial_parameters.kAz)
      oss << element << "; ";
    oss << "]" << std::endl;  
    oss << "kBz : [";
    for (auto element : initial_parameters.kBz)
      oss << element << "; ";
    oss << "]" << std::endl; 
    oss << "Error dead band : " << initial_parameters.error_dead_band << std::endl;
    ROS_INFO("%s", oss.str().c_str());
    output = initial_parameters;
    return true;
  }
  else
  {
    std::string error_string = "Failed to call GetControlLoopParameters"; 
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
}

int main(int argc, char **argv)
{
  // Init the node and get the namespace parameter
  ros::init(argc, argv, "actuator_configuration_example_cpp");

  // For testing purpose
  ros::param::del("/kortex_examples_test_results/actuator_configuration_cpp");

  bool success = true;

  ros::NodeHandle n;
  std::string robot_name = "my_gen3";
  int device_id = 1;

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

  // Parameter device_id
  if (!ros::param::get("~device_id", device_id))
  {
    std::string error_string = "Parameter device_id was not specified, defaulting to " + std::to_string(device_id);
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using device_id " + std::to_string(device_id);
    ROS_INFO("%s", error_string.c_str());
  }

  //-------------------------------------------------------------
  // Find actuators and set which one we want to configure
  success &= example_find_actuators_and_set_device_id(n, robot_name, device_id);

  //-------------------------------------------------------------
  // Clear the faults on a specific actuator
  success &= example_clear_actuator_faults(n, robot_name);

  //-------------------------------------------------------------
  // Get the control loop parameters on a specific actuator
  ControlLoopParameters parameters;
  success &= example_get_control_loop_parameters(n, robot_name, parameters);

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/actuator_configuration_cpp", success);
  
  return success ? 0 : 1;
}