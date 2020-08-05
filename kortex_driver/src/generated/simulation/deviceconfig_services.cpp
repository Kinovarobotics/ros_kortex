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
#include "kortex_driver/generated/simulation/deviceconfig_services.h"

DeviceConfigSimulationServices::DeviceConfigSimulationServices(ros::NodeHandle& node_handle): 
	IDeviceConfigServices(node_handle)
{
	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_SafetyTopic = m_node_handle.advertise<kortex_driver::SafetyNotification>("safety_topic", 1000);
	m_is_activated_SafetyTopic = false;

	m_serviceSetDeviceID = m_node_handle.advertiseService("device_config/set_device_id", &DeviceConfigSimulationServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("device_config/set_api_options", &DeviceConfigSimulationServices::SetApiOptions, this);

	m_serviceGetRunMode = m_node_handle.advertiseService("device_config/get_run_mode", &DeviceConfigSimulationServices::GetRunMode, this);
	m_serviceSetRunMode = m_node_handle.advertiseService("device_config/set_run_mode", &DeviceConfigSimulationServices::SetRunMode, this);
	m_serviceGetDeviceType = m_node_handle.advertiseService("device_config/get_device_type", &DeviceConfigSimulationServices::GetDeviceType, this);
	m_serviceGetFirmwareVersion = m_node_handle.advertiseService("device_config/get_firmware_version", &DeviceConfigSimulationServices::GetFirmwareVersion, this);
	m_serviceGetBootloaderVersion = m_node_handle.advertiseService("device_config/get_bootloader_version", &DeviceConfigSimulationServices::GetBootloaderVersion, this);
	m_serviceGetModelNumber = m_node_handle.advertiseService("device_config/get_model_number", &DeviceConfigSimulationServices::GetModelNumber, this);
	m_serviceGetPartNumber = m_node_handle.advertiseService("device_config/get_part_number", &DeviceConfigSimulationServices::GetPartNumber, this);
	m_serviceGetSerialNumber = m_node_handle.advertiseService("device_config/get_serial_number", &DeviceConfigSimulationServices::GetSerialNumber, this);
	m_serviceGetMACAddress = m_node_handle.advertiseService("device_config/get_m_a_c_address", &DeviceConfigSimulationServices::GetMACAddress, this);
	m_serviceGetIPv4Settings = m_node_handle.advertiseService("device_config/get_i_pv4_settings", &DeviceConfigSimulationServices::GetIPv4Settings, this);
	m_serviceSetIPv4Settings = m_node_handle.advertiseService("device_config/set_i_pv4_settings", &DeviceConfigSimulationServices::SetIPv4Settings, this);
	m_serviceGetPartNumberRevision = m_node_handle.advertiseService("device_config/get_part_number_revision", &DeviceConfigSimulationServices::GetPartNumberRevision, this);
	m_serviceRebootRequest = m_node_handle.advertiseService("device_config/reboot_request", &DeviceConfigSimulationServices::RebootRequest, this);
	m_serviceSetSafetyEnable = m_node_handle.advertiseService("device_config/set_safety_enable", &DeviceConfigSimulationServices::SetSafetyEnable, this);
	m_serviceSetSafetyErrorThreshold = m_node_handle.advertiseService("device_config/set_safety_error_threshold", &DeviceConfigSimulationServices::SetSafetyErrorThreshold, this);
	m_serviceSetSafetyWarningThreshold = m_node_handle.advertiseService("device_config/set_safety_warning_threshold", &DeviceConfigSimulationServices::SetSafetyWarningThreshold, this);
	m_serviceSetSafetyConfiguration = m_node_handle.advertiseService("device_config/set_safety_configuration", &DeviceConfigSimulationServices::SetSafetyConfiguration, this);
	m_serviceGetSafetyConfiguration = m_node_handle.advertiseService("device_config/get_safety_configuration", &DeviceConfigSimulationServices::GetSafetyConfiguration, this);
	m_serviceGetSafetyInformation = m_node_handle.advertiseService("device_config/get_safety_information", &DeviceConfigSimulationServices::GetSafetyInformation, this);
	m_serviceGetSafetyEnable = m_node_handle.advertiseService("device_config/get_safety_enable", &DeviceConfigSimulationServices::GetSafetyEnable, this);
	m_serviceGetSafetyStatus = m_node_handle.advertiseService("device_config/get_safety_status", &DeviceConfigSimulationServices::GetSafetyStatus, this);
	m_serviceClearAllSafetyStatus = m_node_handle.advertiseService("device_config/clear_all_safety_status", &DeviceConfigSimulationServices::ClearAllSafetyStatus, this);
	m_serviceClearSafetyStatus = m_node_handle.advertiseService("device_config/clear_safety_status", &DeviceConfigSimulationServices::ClearSafetyStatus, this);
	m_serviceGetAllSafetyConfiguration = m_node_handle.advertiseService("device_config/get_all_safety_configuration", &DeviceConfigSimulationServices::GetAllSafetyConfiguration, this);
	m_serviceGetAllSafetyInformation = m_node_handle.advertiseService("device_config/get_all_safety_information", &DeviceConfigSimulationServices::GetAllSafetyInformation, this);
	m_serviceResetSafetyDefaults = m_node_handle.advertiseService("device_config/reset_safety_defaults", &DeviceConfigSimulationServices::ResetSafetyDefaults, this);
	m_serviceOnNotificationSafetyTopic = m_node_handle.advertiseService("device_config/activate_publishing_of_safety_topic", &DeviceConfigSimulationServices::OnNotificationSafetyTopic, this);
	m_serviceExecuteCalibration = m_node_handle.advertiseService("device_config/execute_calibration", &DeviceConfigSimulationServices::ExecuteCalibration, this);
	m_serviceGetCalibrationResult = m_node_handle.advertiseService("device_config/get_calibration_result", &DeviceConfigSimulationServices::GetCalibrationResult, this);
	m_serviceStopCalibration = m_node_handle.advertiseService("device_config/stop_calibration", &DeviceConfigSimulationServices::StopCalibration, this);
	m_serviceDeviceConfig_SetCapSenseConfig = m_node_handle.advertiseService("device_config/set_cap_sense_config", &DeviceConfigSimulationServices::DeviceConfig_SetCapSenseConfig, this);
	m_serviceDeviceConfig_GetCapSenseConfig = m_node_handle.advertiseService("device_config/get_cap_sense_config", &DeviceConfigSimulationServices::DeviceConfig_GetCapSenseConfig, this);
}

bool DeviceConfigSimulationServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool DeviceConfigSimulationServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool DeviceConfigSimulationServices::GetRunMode(kortex_driver::GetRunMode::Request  &req, kortex_driver::GetRunMode::Response &res)
{
	
	
	if (GetRunModeHandler)
	{
		res = GetRunModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_run_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetRunMode(kortex_driver::SetRunMode::Request  &req, kortex_driver::SetRunMode::Response &res)
{
	
	
	if (SetRunModeHandler)
	{
		res = SetRunModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_run_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetDeviceType(kortex_driver::GetDeviceType::Request  &req, kortex_driver::GetDeviceType::Response &res)
{
	
	
	if (GetDeviceTypeHandler)
	{
		res = GetDeviceTypeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_device_type is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetFirmwareVersion(kortex_driver::GetFirmwareVersion::Request  &req, kortex_driver::GetFirmwareVersion::Response &res)
{
	
	
	if (GetFirmwareVersionHandler)
	{
		res = GetFirmwareVersionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_firmware_version is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetBootloaderVersion(kortex_driver::GetBootloaderVersion::Request  &req, kortex_driver::GetBootloaderVersion::Response &res)
{
	
	
	if (GetBootloaderVersionHandler)
	{
		res = GetBootloaderVersionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_bootloader_version is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetModelNumber(kortex_driver::GetModelNumber::Request  &req, kortex_driver::GetModelNumber::Response &res)
{
	
	
	if (GetModelNumberHandler)
	{
		res = GetModelNumberHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_model_number is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetPartNumber(kortex_driver::GetPartNumber::Request  &req, kortex_driver::GetPartNumber::Response &res)
{
	
	
	if (GetPartNumberHandler)
	{
		res = GetPartNumberHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_part_number is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSerialNumber(kortex_driver::GetSerialNumber::Request  &req, kortex_driver::GetSerialNumber::Response &res)
{
	
	
	if (GetSerialNumberHandler)
	{
		res = GetSerialNumberHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_serial_number is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetMACAddress(kortex_driver::GetMACAddress::Request  &req, kortex_driver::GetMACAddress::Response &res)
{
	
	
	if (GetMACAddressHandler)
	{
		res = GetMACAddressHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_m_a_c_address is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetIPv4Settings(kortex_driver::GetIPv4Settings::Request  &req, kortex_driver::GetIPv4Settings::Response &res)
{
	
	
	if (GetIPv4SettingsHandler)
	{
		res = GetIPv4SettingsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_i_pv4_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetIPv4Settings(kortex_driver::SetIPv4Settings::Request  &req, kortex_driver::SetIPv4Settings::Response &res)
{
	
	
	if (SetIPv4SettingsHandler)
	{
		res = SetIPv4SettingsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_i_pv4_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetPartNumberRevision(kortex_driver::GetPartNumberRevision::Request  &req, kortex_driver::GetPartNumberRevision::Response &res)
{
	
	
	if (GetPartNumberRevisionHandler)
	{
		res = GetPartNumberRevisionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_part_number_revision is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::RebootRequest(kortex_driver::RebootRequest::Request  &req, kortex_driver::RebootRequest::Response &res)
{
	
	
	if (RebootRequestHandler)
	{
		res = RebootRequestHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/reboot_request is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyEnable(kortex_driver::SetSafetyEnable::Request  &req, kortex_driver::SetSafetyEnable::Response &res)
{
	
	
	if (SetSafetyEnableHandler)
	{
		res = SetSafetyEnableHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_safety_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyErrorThreshold(kortex_driver::SetSafetyErrorThreshold::Request  &req, kortex_driver::SetSafetyErrorThreshold::Response &res)
{
	
	
	if (SetSafetyErrorThresholdHandler)
	{
		res = SetSafetyErrorThresholdHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_safety_error_threshold is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyWarningThreshold(kortex_driver::SetSafetyWarningThreshold::Request  &req, kortex_driver::SetSafetyWarningThreshold::Response &res)
{
	
	
	if (SetSafetyWarningThresholdHandler)
	{
		res = SetSafetyWarningThresholdHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_safety_warning_threshold is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyConfiguration(kortex_driver::SetSafetyConfiguration::Request  &req, kortex_driver::SetSafetyConfiguration::Response &res)
{
	
	
	if (SetSafetyConfigurationHandler)
	{
		res = SetSafetyConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_safety_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyConfiguration(kortex_driver::GetSafetyConfiguration::Request  &req, kortex_driver::GetSafetyConfiguration::Response &res)
{
	
	
	if (GetSafetyConfigurationHandler)
	{
		res = GetSafetyConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_safety_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyInformation(kortex_driver::GetSafetyInformation::Request  &req, kortex_driver::GetSafetyInformation::Response &res)
{
	
	
	if (GetSafetyInformationHandler)
	{
		res = GetSafetyInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_safety_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyEnable(kortex_driver::GetSafetyEnable::Request  &req, kortex_driver::GetSafetyEnable::Response &res)
{
	
	
	if (GetSafetyEnableHandler)
	{
		res = GetSafetyEnableHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_safety_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyStatus(kortex_driver::GetSafetyStatus::Request  &req, kortex_driver::GetSafetyStatus::Response &res)
{
	
	
	if (GetSafetyStatusHandler)
	{
		res = GetSafetyStatusHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_safety_status is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::ClearAllSafetyStatus(kortex_driver::ClearAllSafetyStatus::Request  &req, kortex_driver::ClearAllSafetyStatus::Response &res)
{
	
	
	if (ClearAllSafetyStatusHandler)
	{
		res = ClearAllSafetyStatusHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/clear_all_safety_status is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::ClearSafetyStatus(kortex_driver::ClearSafetyStatus::Request  &req, kortex_driver::ClearSafetyStatus::Response &res)
{
	
	
	if (ClearSafetyStatusHandler)
	{
		res = ClearSafetyStatusHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/clear_safety_status is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetAllSafetyConfiguration(kortex_driver::GetAllSafetyConfiguration::Request  &req, kortex_driver::GetAllSafetyConfiguration::Response &res)
{
	
	
	if (GetAllSafetyConfigurationHandler)
	{
		res = GetAllSafetyConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_all_safety_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetAllSafetyInformation(kortex_driver::GetAllSafetyInformation::Request  &req, kortex_driver::GetAllSafetyInformation::Response &res)
{
	
	
	if (GetAllSafetyInformationHandler)
	{
		res = GetAllSafetyInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_all_safety_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::ResetSafetyDefaults(kortex_driver::ResetSafetyDefaults::Request  &req, kortex_driver::ResetSafetyDefaults::Response &res)
{
	
	
	if (ResetSafetyDefaultsHandler)
	{
		res = ResetSafetyDefaultsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/reset_safety_defaults is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::OnNotificationSafetyTopic(kortex_driver::OnNotificationSafetyTopic::Request  &req, kortex_driver::OnNotificationSafetyTopic::Response &res)
{
	
	m_is_activated_SafetyTopic = true;
	
	if (OnNotificationSafetyTopicHandler)
	{
		res = OnNotificationSafetyTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/activate_publishing_of_safety_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void DeviceConfigSimulationServices::cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif)
{
	kortex_driver::SafetyNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SafetyTopic.publish(ros_msg);
}

bool DeviceConfigSimulationServices::ExecuteCalibration(kortex_driver::ExecuteCalibration::Request  &req, kortex_driver::ExecuteCalibration::Response &res)
{
	
	
	if (ExecuteCalibrationHandler)
	{
		res = ExecuteCalibrationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/execute_calibration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetCalibrationResult(kortex_driver::GetCalibrationResult::Request  &req, kortex_driver::GetCalibrationResult::Response &res)
{
	
	
	if (GetCalibrationResultHandler)
	{
		res = GetCalibrationResultHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_calibration_result is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::StopCalibration(kortex_driver::StopCalibration::Request  &req, kortex_driver::StopCalibration::Response &res)
{
	
	
	if (StopCalibrationHandler)
	{
		res = StopCalibrationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/stop_calibration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::DeviceConfig_SetCapSenseConfig(kortex_driver::DeviceConfig_SetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_SetCapSenseConfig::Response &res)
{
	
	
	if (DeviceConfig_SetCapSenseConfigHandler)
	{
		res = DeviceConfig_SetCapSenseConfigHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/set_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::DeviceConfig_GetCapSenseConfig(kortex_driver::DeviceConfig_GetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_GetCapSenseConfig::Response &res)
{
	
	
	if (DeviceConfig_GetCapSenseConfigHandler)
	{
		res = DeviceConfig_GetCapSenseConfigHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for device_config/get_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}
