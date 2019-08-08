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
 
#include "kortex_driver/generated/common_proto_converter.h"
#include "kortex_driver/generated/common_ros_converter.h"
#include "kortex_driver/generated/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/base_proto_converter.h"
#include "kortex_driver/generated/base_ros_converter.h"
#include "kortex_driver/generated/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/basecyclic_proto_converter.h"
#include "kortex_driver/generated/basecyclic_ros_converter.h"
#include "kortex_driver/generated/controlconfig_proto_converter.h"
#include "kortex_driver/generated/controlconfig_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/devicemanager_proto_converter.h"
#include "kortex_driver/generated/devicemanager_ros_converter.h"
#include "kortex_driver/generated/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/visionconfig_proto_converter.h"
#include "kortex_driver/generated/visionconfig_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_services.h"

DeviceConfigServices::DeviceConfigServices(ros::NodeHandle& n, Kinova::Api::DeviceConfig::DeviceConfigClient* deviceconfig, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_deviceconfig(deviceconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_SafetyTopic = m_n.advertise<kortex_driver::SafetyNotification>("safety_topic", 1000);

	m_serviceSetDeviceID = n.advertiseService("device_config/set_device_id", &DeviceConfigServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("device_config/set_api_options", &DeviceConfigServices::SetApiOptions, this);

	m_serviceGetRunMode = m_n.advertiseService("device_config/get_run_mode", &DeviceConfigServices::GetRunMode, this);
	m_serviceSetRunMode = m_n.advertiseService("device_config/set_run_mode", &DeviceConfigServices::SetRunMode, this);
	m_serviceGetDeviceType = m_n.advertiseService("device_config/get_device_type", &DeviceConfigServices::GetDeviceType, this);
	m_serviceGetFirmwareVersion = m_n.advertiseService("device_config/get_firmware_version", &DeviceConfigServices::GetFirmwareVersion, this);
	m_serviceGetBootloaderVersion = m_n.advertiseService("device_config/get_bootloader_version", &DeviceConfigServices::GetBootloaderVersion, this);
	m_serviceGetModelNumber = m_n.advertiseService("device_config/get_model_number", &DeviceConfigServices::GetModelNumber, this);
	m_serviceGetPartNumber = m_n.advertiseService("device_config/get_part_number", &DeviceConfigServices::GetPartNumber, this);
	m_serviceGetSerialNumber = m_n.advertiseService("device_config/get_serial_number", &DeviceConfigServices::GetSerialNumber, this);
	m_serviceGetMACAddress = m_n.advertiseService("device_config/get_m_a_c_address", &DeviceConfigServices::GetMACAddress, this);
	m_serviceGetIPv4Settings = m_n.advertiseService("device_config/get_i_pv4_settings", &DeviceConfigServices::GetIPv4Settings, this);
	m_serviceSetIPv4Settings = m_n.advertiseService("device_config/set_i_pv4_settings", &DeviceConfigServices::SetIPv4Settings, this);
	m_serviceGetPartNumberRevision = m_n.advertiseService("device_config/get_part_number_revision", &DeviceConfigServices::GetPartNumberRevision, this);
	m_serviceRebootRequest = m_n.advertiseService("device_config/reboot_request", &DeviceConfigServices::RebootRequest, this);
	m_serviceSetSafetyEnable = m_n.advertiseService("device_config/set_safety_enable", &DeviceConfigServices::SetSafetyEnable, this);
	m_serviceSetSafetyErrorThreshold = m_n.advertiseService("device_config/set_safety_error_threshold", &DeviceConfigServices::SetSafetyErrorThreshold, this);
	m_serviceSetSafetyWarningThreshold = m_n.advertiseService("device_config/set_safety_warning_threshold", &DeviceConfigServices::SetSafetyWarningThreshold, this);
	m_serviceSetSafetyConfiguration = m_n.advertiseService("device_config/set_safety_configuration", &DeviceConfigServices::SetSafetyConfiguration, this);
	m_serviceGetSafetyConfiguration = m_n.advertiseService("device_config/get_safety_configuration", &DeviceConfigServices::GetSafetyConfiguration, this);
	m_serviceGetSafetyInformation = m_n.advertiseService("device_config/get_safety_information", &DeviceConfigServices::GetSafetyInformation, this);
	m_serviceGetSafetyEnable = m_n.advertiseService("device_config/get_safety_enable", &DeviceConfigServices::GetSafetyEnable, this);
	m_serviceGetSafetyStatus = m_n.advertiseService("device_config/get_safety_status", &DeviceConfigServices::GetSafetyStatus, this);
	m_serviceClearAllSafetyStatus = m_n.advertiseService("device_config/clear_all_safety_status", &DeviceConfigServices::ClearAllSafetyStatus, this);
	m_serviceClearSafetyStatus = m_n.advertiseService("device_config/clear_safety_status", &DeviceConfigServices::ClearSafetyStatus, this);
	m_serviceGetAllSafetyConfiguration = m_n.advertiseService("device_config/get_all_safety_configuration", &DeviceConfigServices::GetAllSafetyConfiguration, this);
	m_serviceGetAllSafetyInformation = m_n.advertiseService("device_config/get_all_safety_information", &DeviceConfigServices::GetAllSafetyInformation, this);
	m_serviceResetSafetyDefaults = m_n.advertiseService("device_config/reset_safety_defaults", &DeviceConfigServices::ResetSafetyDefaults, this);
	m_serviceOnNotificationSafetyTopic = m_n.advertiseService("device_config/activate_publishing_of_safety_topic", &DeviceConfigServices::OnNotificationSafetyTopic, this);
	m_serviceExecuteCalibration = m_n.advertiseService("device_config/execute_calibration", &DeviceConfigServices::ExecuteCalibration, this);
	m_serviceGetCalibrationResult = m_n.advertiseService("device_config/get_calibration_result", &DeviceConfigServices::GetCalibrationResult, this);
	m_serviceStopCalibration = m_n.advertiseService("device_config/stop_calibration", &DeviceConfigServices::StopCalibration, this);
	m_serviceDeviceConfig_SetCapSenseConfig = m_n.advertiseService("device_config/set_cap_sense_config", &DeviceConfigServices::DeviceConfig_SetCapSenseConfig, this);
	m_serviceDeviceConfig_GetCapSenseConfig = m_n.advertiseService("device_config/get_cap_sense_config", &DeviceConfigServices::DeviceConfig_GetCapSenseConfig, this);
}

bool DeviceConfigServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool DeviceConfigServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool DeviceConfigServices::GetRunMode(kortex_driver::GetRunMode::Request  &req, kortex_driver::GetRunMode::Response &res)
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

bool DeviceConfigServices::SetRunMode(kortex_driver::SetRunMode::Request  &req, kortex_driver::SetRunMode::Response &res)
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

bool DeviceConfigServices::GetDeviceType(kortex_driver::GetDeviceType::Request  &req, kortex_driver::GetDeviceType::Response &res)
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

bool DeviceConfigServices::GetFirmwareVersion(kortex_driver::GetFirmwareVersion::Request  &req, kortex_driver::GetFirmwareVersion::Response &res)
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

bool DeviceConfigServices::GetBootloaderVersion(kortex_driver::GetBootloaderVersion::Request  &req, kortex_driver::GetBootloaderVersion::Response &res)
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

bool DeviceConfigServices::GetModelNumber(kortex_driver::GetModelNumber::Request  &req, kortex_driver::GetModelNumber::Response &res)
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

bool DeviceConfigServices::GetPartNumber(kortex_driver::GetPartNumber::Request  &req, kortex_driver::GetPartNumber::Response &res)
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

bool DeviceConfigServices::GetSerialNumber(kortex_driver::GetSerialNumber::Request  &req, kortex_driver::GetSerialNumber::Response &res)
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

bool DeviceConfigServices::GetMACAddress(kortex_driver::GetMACAddress::Request  &req, kortex_driver::GetMACAddress::Response &res)
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

bool DeviceConfigServices::GetIPv4Settings(kortex_driver::GetIPv4Settings::Request  &req, kortex_driver::GetIPv4Settings::Response &res)
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

bool DeviceConfigServices::SetIPv4Settings(kortex_driver::SetIPv4Settings::Request  &req, kortex_driver::SetIPv4Settings::Response &res)
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

bool DeviceConfigServices::GetPartNumberRevision(kortex_driver::GetPartNumberRevision::Request  &req, kortex_driver::GetPartNumberRevision::Response &res)
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

bool DeviceConfigServices::RebootRequest(kortex_driver::RebootRequest::Request  &req, kortex_driver::RebootRequest::Response &res)
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

bool DeviceConfigServices::SetSafetyEnable(kortex_driver::SetSafetyEnable::Request  &req, kortex_driver::SetSafetyEnable::Response &res)
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

bool DeviceConfigServices::SetSafetyErrorThreshold(kortex_driver::SetSafetyErrorThreshold::Request  &req, kortex_driver::SetSafetyErrorThreshold::Response &res)
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

bool DeviceConfigServices::SetSafetyWarningThreshold(kortex_driver::SetSafetyWarningThreshold::Request  &req, kortex_driver::SetSafetyWarningThreshold::Response &res)
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

bool DeviceConfigServices::SetSafetyConfiguration(kortex_driver::SetSafetyConfiguration::Request  &req, kortex_driver::SetSafetyConfiguration::Response &res)
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

bool DeviceConfigServices::GetSafetyConfiguration(kortex_driver::GetSafetyConfiguration::Request  &req, kortex_driver::GetSafetyConfiguration::Response &res)
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

bool DeviceConfigServices::GetSafetyInformation(kortex_driver::GetSafetyInformation::Request  &req, kortex_driver::GetSafetyInformation::Response &res)
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

bool DeviceConfigServices::GetSafetyEnable(kortex_driver::GetSafetyEnable::Request  &req, kortex_driver::GetSafetyEnable::Response &res)
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

bool DeviceConfigServices::GetSafetyStatus(kortex_driver::GetSafetyStatus::Request  &req, kortex_driver::GetSafetyStatus::Response &res)
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

bool DeviceConfigServices::ClearAllSafetyStatus(kortex_driver::ClearAllSafetyStatus::Request  &req, kortex_driver::ClearAllSafetyStatus::Response &res)
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

bool DeviceConfigServices::ClearSafetyStatus(kortex_driver::ClearSafetyStatus::Request  &req, kortex_driver::ClearSafetyStatus::Response &res)
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

bool DeviceConfigServices::GetAllSafetyConfiguration(kortex_driver::GetAllSafetyConfiguration::Request  &req, kortex_driver::GetAllSafetyConfiguration::Response &res)
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

bool DeviceConfigServices::GetAllSafetyInformation(kortex_driver::GetAllSafetyInformation::Request  &req, kortex_driver::GetAllSafetyInformation::Response &res)
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

bool DeviceConfigServices::ResetSafetyDefaults(kortex_driver::ResetSafetyDefaults::Request  &req, kortex_driver::ResetSafetyDefaults::Response &res)
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

bool DeviceConfigServices::OnNotificationSafetyTopic(kortex_driver::OnNotificationSafetyTopic::Request  &req, kortex_driver::OnNotificationSafetyTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Common::SafetyNotification) > callback = std::bind(&DeviceConfigServices::cb_SafetyTopic, this, std::placeholders::_1);
		output = m_deviceconfig->OnNotificationSafetyTopic(callback, input, m_current_device_id);
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
void DeviceConfigServices::cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif)
{
	kortex_driver::SafetyNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SafetyTopic.publish(ros_msg);
}

bool DeviceConfigServices::ExecuteCalibration(kortex_driver::ExecuteCalibration::Request  &req, kortex_driver::ExecuteCalibration::Response &res)
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

bool DeviceConfigServices::GetCalibrationResult(kortex_driver::GetCalibrationResult::Request  &req, kortex_driver::GetCalibrationResult::Response &res)
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

bool DeviceConfigServices::StopCalibration(kortex_driver::StopCalibration::Request  &req, kortex_driver::StopCalibration::Response &res)
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

bool DeviceConfigServices::DeviceConfig_SetCapSenseConfig(kortex_driver::DeviceConfig_SetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_SetCapSenseConfig::Response &res)
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

bool DeviceConfigServices::DeviceConfig_GetCapSenseConfig(kortex_driver::DeviceConfig_GetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_GetCapSenseConfig::Response &res)
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
