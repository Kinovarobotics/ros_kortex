/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/*
 * This file has been auto-generated and should not be modified.
 */
 
#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_services.h"

DeviceConfigRobotServices::DeviceConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::DeviceConfig::DeviceConfigClient* deviceconfig, uint32_t device_id, uint32_t timeout_ms): 
	IDeviceConfigServices(node_handle),
	m_deviceconfig(deviceconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_SafetyTopic = m_node_handle.advertise<kortex_driver::SafetyNotification>("safety_topic", 1000);
	m_is_activated_SafetyTopic = false;

	m_serviceSetDeviceID = m_node_handle.advertiseService("device_config/set_device_id", &DeviceConfigRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("device_config/set_api_options", &DeviceConfigRobotServices::SetApiOptions, this);

	m_serviceGetRunMode = m_node_handle.advertiseService("device_config/get_run_mode", &DeviceConfigRobotServices::GetRunMode, this);
	m_serviceSetRunMode = m_node_handle.advertiseService("device_config/set_run_mode", &DeviceConfigRobotServices::SetRunMode, this);
	m_serviceGetDeviceType = m_node_handle.advertiseService("device_config/get_device_type", &DeviceConfigRobotServices::GetDeviceType, this);
	m_serviceGetFirmwareVersion = m_node_handle.advertiseService("device_config/get_firmware_version", &DeviceConfigRobotServices::GetFirmwareVersion, this);
	m_serviceGetBootloaderVersion = m_node_handle.advertiseService("device_config/get_bootloader_version", &DeviceConfigRobotServices::GetBootloaderVersion, this);
	m_serviceGetModelNumber = m_node_handle.advertiseService("device_config/get_model_number", &DeviceConfigRobotServices::GetModelNumber, this);
	m_serviceGetPartNumber = m_node_handle.advertiseService("device_config/get_part_number", &DeviceConfigRobotServices::GetPartNumber, this);
	m_serviceGetSerialNumber = m_node_handle.advertiseService("device_config/get_serial_number", &DeviceConfigRobotServices::GetSerialNumber, this);
	m_serviceGetMACAddress = m_node_handle.advertiseService("device_config/get_m_a_c_address", &DeviceConfigRobotServices::GetMACAddress, this);
	m_serviceGetIPv4Settings = m_node_handle.advertiseService("device_config/get_i_pv4_settings", &DeviceConfigRobotServices::GetIPv4Settings, this);
	m_serviceSetIPv4Settings = m_node_handle.advertiseService("device_config/set_i_pv4_settings", &DeviceConfigRobotServices::SetIPv4Settings, this);
	m_serviceGetPartNumberRevision = m_node_handle.advertiseService("device_config/get_part_number_revision", &DeviceConfigRobotServices::GetPartNumberRevision, this);
	m_serviceRebootRequest = m_node_handle.advertiseService("device_config/reboot_request", &DeviceConfigRobotServices::RebootRequest, this);
	m_serviceSetSafetyEnable = m_node_handle.advertiseService("device_config/set_safety_enable", &DeviceConfigRobotServices::SetSafetyEnable, this);
	m_serviceSetSafetyErrorThreshold = m_node_handle.advertiseService("device_config/set_safety_error_threshold", &DeviceConfigRobotServices::SetSafetyErrorThreshold, this);
	m_serviceSetSafetyWarningThreshold = m_node_handle.advertiseService("device_config/set_safety_warning_threshold", &DeviceConfigRobotServices::SetSafetyWarningThreshold, this);
	m_serviceSetSafetyConfiguration = m_node_handle.advertiseService("device_config/set_safety_configuration", &DeviceConfigRobotServices::SetSafetyConfiguration, this);
	m_serviceGetSafetyConfiguration = m_node_handle.advertiseService("device_config/get_safety_configuration", &DeviceConfigRobotServices::GetSafetyConfiguration, this);
	m_serviceGetSafetyInformation = m_node_handle.advertiseService("device_config/get_safety_information", &DeviceConfigRobotServices::GetSafetyInformation, this);
	m_serviceGetSafetyEnable = m_node_handle.advertiseService("device_config/get_safety_enable", &DeviceConfigRobotServices::GetSafetyEnable, this);
	m_serviceGetSafetyStatus = m_node_handle.advertiseService("device_config/get_safety_status", &DeviceConfigRobotServices::GetSafetyStatus, this);
	m_serviceClearAllSafetyStatus = m_node_handle.advertiseService("device_config/clear_all_safety_status", &DeviceConfigRobotServices::ClearAllSafetyStatus, this);
	m_serviceClearSafetyStatus = m_node_handle.advertiseService("device_config/clear_safety_status", &DeviceConfigRobotServices::ClearSafetyStatus, this);
	m_serviceGetAllSafetyConfiguration = m_node_handle.advertiseService("device_config/get_all_safety_configuration", &DeviceConfigRobotServices::GetAllSafetyConfiguration, this);
	m_serviceGetAllSafetyInformation = m_node_handle.advertiseService("device_config/get_all_safety_information", &DeviceConfigRobotServices::GetAllSafetyInformation, this);
	m_serviceResetSafetyDefaults = m_node_handle.advertiseService("device_config/reset_safety_defaults", &DeviceConfigRobotServices::ResetSafetyDefaults, this);
	m_serviceOnNotificationSafetyTopic = m_node_handle.advertiseService("device_config/activate_publishing_of_safety_topic", &DeviceConfigRobotServices::OnNotificationSafetyTopic, this);
	m_serviceExecuteCalibration = m_node_handle.advertiseService("device_config/execute_calibration", &DeviceConfigRobotServices::ExecuteCalibration, this);
	m_serviceGetCalibrationResult = m_node_handle.advertiseService("device_config/get_calibration_result", &DeviceConfigRobotServices::GetCalibrationResult, this);
	m_serviceStopCalibration = m_node_handle.advertiseService("device_config/stop_calibration", &DeviceConfigRobotServices::StopCalibration, this);
	m_serviceDeviceConfig_SetCapSenseConfig = m_node_handle.advertiseService("device_config/set_cap_sense_config", &DeviceConfigRobotServices::DeviceConfig_SetCapSenseConfig, this);
	m_serviceDeviceConfig_GetCapSenseConfig = m_node_handle.advertiseService("device_config/get_cap_sense_config", &DeviceConfigRobotServices::DeviceConfig_GetCapSenseConfig, this);
}

bool DeviceConfigRobotServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool DeviceConfigRobotServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool DeviceConfigRobotServices::GetRunMode(kortex_driver::GetRunMode::Request  &req, kortex_driver::GetRunMode::Response &res)
{
	
	Kinova::Api::DeviceConfig::RunMode output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetRunMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::SetRunMode(kortex_driver::SetRunMode::Request  &req, kortex_driver::SetRunMode::Response &res)
{
	
	Kinova::Api::DeviceConfig::RunMode input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetRunMode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::GetDeviceType(kortex_driver::GetDeviceType::Request  &req, kortex_driver::GetDeviceType::Response &res)
{
	
	Kinova::Api::DeviceConfig::DeviceType output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetDeviceType(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetFirmwareVersion(kortex_driver::GetFirmwareVersion::Request  &req, kortex_driver::GetFirmwareVersion::Response &res)
{
	
	Kinova::Api::DeviceConfig::FirmwareVersion output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetFirmwareVersion(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetBootloaderVersion(kortex_driver::GetBootloaderVersion::Request  &req, kortex_driver::GetBootloaderVersion::Response &res)
{
	
	Kinova::Api::DeviceConfig::BootloaderVersion output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetBootloaderVersion(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetModelNumber(kortex_driver::GetModelNumber::Request  &req, kortex_driver::GetModelNumber::Response &res)
{
	
	Kinova::Api::DeviceConfig::ModelNumber output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetModelNumber(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetPartNumber(kortex_driver::GetPartNumber::Request  &req, kortex_driver::GetPartNumber::Response &res)
{
	
	Kinova::Api::DeviceConfig::PartNumber output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPartNumber(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSerialNumber(kortex_driver::GetSerialNumber::Request  &req, kortex_driver::GetSerialNumber::Response &res)
{
	
	Kinova::Api::DeviceConfig::SerialNumber output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSerialNumber(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetMACAddress(kortex_driver::GetMACAddress::Request  &req, kortex_driver::GetMACAddress::Response &res)
{
	
	Kinova::Api::DeviceConfig::MACAddress output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetMACAddress(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetIPv4Settings(kortex_driver::GetIPv4Settings::Request  &req, kortex_driver::GetIPv4Settings::Response &res)
{
	
	Kinova::Api::DeviceConfig::IPv4Settings output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetIPv4Settings(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::SetIPv4Settings(kortex_driver::SetIPv4Settings::Request  &req, kortex_driver::SetIPv4Settings::Response &res)
{
	
	Kinova::Api::DeviceConfig::IPv4Settings input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetIPv4Settings(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::GetPartNumberRevision(kortex_driver::GetPartNumberRevision::Request  &req, kortex_driver::GetPartNumberRevision::Response &res)
{
	
	Kinova::Api::DeviceConfig::PartNumberRevision output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPartNumberRevision(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::RebootRequest(kortex_driver::RebootRequest::Request  &req, kortex_driver::RebootRequest::Response &res)
{
	
	Kinova::Api::DeviceConfig::RebootRqst input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->RebootRequest(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::SetSafetyEnable(kortex_driver::SetSafetyEnable::Request  &req, kortex_driver::SetSafetyEnable::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyEnable input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyEnable(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::SetSafetyErrorThreshold(kortex_driver::SetSafetyErrorThreshold::Request  &req, kortex_driver::SetSafetyErrorThreshold::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyThreshold input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyErrorThreshold(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::SetSafetyWarningThreshold(kortex_driver::SetSafetyWarningThreshold::Request  &req, kortex_driver::SetSafetyWarningThreshold::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyThreshold input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyWarningThreshold(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::SetSafetyConfiguration(kortex_driver::SetSafetyConfiguration::Request  &req, kortex_driver::SetSafetyConfiguration::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::GetSafetyConfiguration(kortex_driver::GetSafetyConfiguration::Request  &req, kortex_driver::GetSafetyConfiguration::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSafetyInformation(kortex_driver::GetSafetyInformation::Request  &req, kortex_driver::GetSafetyInformation::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyInformation(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSafetyEnable(kortex_driver::GetSafetyEnable::Request  &req, kortex_driver::GetSafetyEnable::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyEnable output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyEnable(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSafetyStatus(kortex_driver::GetSafetyStatus::Request  &req, kortex_driver::GetSafetyStatus::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyStatus output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyStatus(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::ClearAllSafetyStatus(kortex_driver::ClearAllSafetyStatus::Request  &req, kortex_driver::ClearAllSafetyStatus::Response &res)
{
	
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->ClearAllSafetyStatus(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::ClearSafetyStatus(kortex_driver::ClearSafetyStatus::Request  &req, kortex_driver::ClearSafetyStatus::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->ClearSafetyStatus(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::GetAllSafetyConfiguration(kortex_driver::GetAllSafetyConfiguration::Request  &req, kortex_driver::GetAllSafetyConfiguration::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyConfigurationList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetAllSafetyConfiguration(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetAllSafetyInformation(kortex_driver::GetAllSafetyInformation::Request  &req, kortex_driver::GetAllSafetyInformation::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyInformationList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetAllSafetyInformation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::ResetSafetyDefaults(kortex_driver::ResetSafetyDefaults::Request  &req, kortex_driver::ResetSafetyDefaults::Response &res)
{
	
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->ResetSafetyDefaults(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::OnNotificationSafetyTopic(kortex_driver::OnNotificationSafetyTopic::Request  &req, kortex_driver::OnNotificationSafetyTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_SafetyTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Common::SafetyNotification) > callback = std::bind(&DeviceConfigRobotServices::cb_SafetyTopic, this, std::placeholders::_1);
		output = m_deviceconfig->OnNotificationSafetyTopic(callback, input, m_current_device_id);
		m_is_activated_SafetyTopic = true;
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void DeviceConfigRobotServices::cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif)
{
	kortex_driver::SafetyNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SafetyTopic.publish(ros_msg);
}

bool DeviceConfigRobotServices::ExecuteCalibration(kortex_driver::ExecuteCalibration::Request  &req, kortex_driver::ExecuteCalibration::Response &res)
{
	
	Kinova::Api::DeviceConfig::Calibration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->ExecuteCalibration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::GetCalibrationResult(kortex_driver::GetCalibrationResult::Request  &req, kortex_driver::GetCalibrationResult::Response &res)
{
	
	Kinova::Api::DeviceConfig::CalibrationElement input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::CalibrationResult output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetCalibrationResult(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::StopCalibration(kortex_driver::StopCalibration::Request  &req, kortex_driver::StopCalibration::Response &res)
{
	
	Kinova::Api::DeviceConfig::Calibration input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::CalibrationResult output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->StopCalibration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::DeviceConfig_SetCapSenseConfig(kortex_driver::DeviceConfig_SetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_SetCapSenseConfig::Response &res)
{
	
	Kinova::Api::DeviceConfig::CapSenseConfig input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetCapSenseConfig(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool DeviceConfigRobotServices::DeviceConfig_GetCapSenseConfig(kortex_driver::DeviceConfig_GetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_GetCapSenseConfig::Response &res)
{
	
	Kinova::Api::DeviceConfig::CapSenseConfig output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetCapSenseConfig(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
