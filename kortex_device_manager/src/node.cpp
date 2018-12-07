/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
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
 
#include "node.h"
#include "common_ros_converter.h"
#include "common_proto_converter.h"
#include "deviceconfig_ros_converter.h"
#include "deviceconfig_proto_converter.h"
#include "devicemanager_ros_converter.h"
#include "devicemanager_proto_converter.h"
KortexDeviceManager::KortexDeviceManager(char* ip, ros::NodeHandle& n) : m_n(n)
{
	m_transport = new TransportClientUdp();
	m_transport->connect(ip, 10000);

	m_router = new RouterClient(m_transport, [](KError err) { cout << "_________ callback error _________" << err.toString(); });


	m_deviceconfig = new DeviceConfig::DeviceConfigClient(m_router);
	m_devicemanager = new DeviceManager::DeviceManagerClient(m_router);m_SessionManager = new SessionManager(m_router);
	auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
	
	createSessionInfo.set_username("admin");
	createSessionInfo.set_password("admin");
	createSessionInfo.set_session_inactivity_timeout(35000);

	m_SessionManager->CreateSession(createSessionInfo);
	std::cout << "\nSession Created\n";

	m_pub_Error = m_n.advertise<kortex_device_manager::KortexError>("KortexError", 1000);
	m_pub_SafetyTopic = m_n.advertise<kortex_device_manager::SafetyNotification>("SafetyTopic", 1000);std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}




bool KortexDeviceManager::GetRunMode(kortex_device_manager::GetRunMode::Request  &req, kortex_device_manager::GetRunMode::Response &res)
{
	Empty input;
	RunMode output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetRunMode();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::SetRunMode(kortex_device_manager::SetRunMode::Request  &req, kortex_device_manager::SetRunMode::Response &res)
{
	RunMode input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetRunMode(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::GetDeviceType(kortex_device_manager::GetDeviceType::Request  &req, kortex_device_manager::GetDeviceType::Response &res)
{
	Empty input;
	DeviceType output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetDeviceType();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetFirmwareVersion(kortex_device_manager::GetFirmwareVersion::Request  &req, kortex_device_manager::GetFirmwareVersion::Response &res)
{
	Empty input;
	FirmwareVersion output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetFirmwareVersion();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetBootloaderVersion(kortex_device_manager::GetBootloaderVersion::Request  &req, kortex_device_manager::GetBootloaderVersion::Response &res)
{
	Empty input;
	BootloaderVersion output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetBootloaderVersion();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetModelNumber(kortex_device_manager::GetModelNumber::Request  &req, kortex_device_manager::GetModelNumber::Response &res)
{
	Empty input;
	ModelNumber output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetModelNumber();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetPartNumber(kortex_device_manager::GetPartNumber::Request  &req, kortex_device_manager::GetPartNumber::Response &res)
{
	Empty input;
	PartNumber output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPartNumber();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetSerialNumber(kortex_device_manager::GetSerialNumber::Request  &req, kortex_device_manager::GetSerialNumber::Response &res)
{
	Empty input;
	SerialNumber output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSerialNumber();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetMACAddress(kortex_device_manager::GetMACAddress::Request  &req, kortex_device_manager::GetMACAddress::Response &res)
{
	Empty input;
	MACAddress output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetMACAddress();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetIPv4Settings(kortex_device_manager::GetIPv4Settings::Request  &req, kortex_device_manager::GetIPv4Settings::Response &res)
{
	Empty input;
	IPv4Settings output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetIPv4Settings();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::SetIPv4Settings(kortex_device_manager::SetIPv4Settings::Request  &req, kortex_device_manager::SetIPv4Settings::Response &res)
{
	IPv4Settings input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetIPv4Settings(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::GetPartNumberRevision(kortex_device_manager::GetPartNumberRevision::Request  &req, kortex_device_manager::GetPartNumberRevision::Response &res)
{
	Empty input;
	PartNumberRevision output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPartNumberRevision();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetPowerOnSelfTestResult(kortex_device_manager::GetPowerOnSelfTestResult::Request  &req, kortex_device_manager::GetPowerOnSelfTestResult::Response &res)
{
	Empty input;
	PowerOnSelfTestResult output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPowerOnSelfTestResult();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::RebootRequest(kortex_device_manager::RebootRequest::Request  &req, kortex_device_manager::RebootRequest::Response &res)
{
	RebootRqst input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->RebootRequest(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetSafetyEnable(kortex_device_manager::SetSafetyEnable::Request  &req, kortex_device_manager::SetSafetyEnable::Response &res)
{
	SafetyEnable input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyEnable(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetSafetyErrorThreshold(kortex_device_manager::SetSafetyErrorThreshold::Request  &req, kortex_device_manager::SetSafetyErrorThreshold::Response &res)
{
	SafetyThreshold input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyErrorThreshold(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetSafetyWarningThreshold(kortex_device_manager::SetSafetyWarningThreshold::Request  &req, kortex_device_manager::SetSafetyWarningThreshold::Response &res)
{
	SafetyThreshold input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyWarningThreshold(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetSafetyConfiguration(kortex_device_manager::SetSafetyConfiguration::Request  &req, kortex_device_manager::SetSafetyConfiguration::Response &res)
{
	SafetyConfiguration input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyConfiguration(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::GetSafetyConfiguration(kortex_device_manager::GetSafetyConfiguration::Request  &req, kortex_device_manager::GetSafetyConfiguration::Response &res)
{
	SafetyHandle input;
	ToProtoData(req.input, &input);
	SafetyConfiguration output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyConfiguration(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetSafetyInformation(kortex_device_manager::GetSafetyInformation::Request  &req, kortex_device_manager::GetSafetyInformation::Response &res)
{
	SafetyHandle input;
	ToProtoData(req.input, &input);
	SafetyInformation output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyInformation(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetSafetyEnable(kortex_device_manager::GetSafetyEnable::Request  &req, kortex_device_manager::GetSafetyEnable::Response &res)
{
	SafetyHandle input;
	ToProtoData(req.input, &input);
	SafetyEnable output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyEnable(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetSafetyStatus(kortex_device_manager::GetSafetyStatus::Request  &req, kortex_device_manager::GetSafetyStatus::Response &res)
{
	SafetyHandle input;
	ToProtoData(req.input, &input);
	SafetyStatus output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyStatus(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::ClearAllSafetyStatus(kortex_device_manager::ClearAllSafetyStatus::Request  &req, kortex_device_manager::ClearAllSafetyStatus::Response &res)
{
	Empty input;
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->ClearAllSafetyStatus();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::ClearSafetyStatus(kortex_device_manager::ClearSafetyStatus::Request  &req, kortex_device_manager::ClearSafetyStatus::Response &res)
{
	SafetyHandle input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->ClearSafetyStatus(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::GetAllSafetyConfiguration(kortex_device_manager::GetAllSafetyConfiguration::Request  &req, kortex_device_manager::GetAllSafetyConfiguration::Response &res)
{
	Empty input;
	SafetyConfigurationList output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetAllSafetyConfiguration();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::GetAllSafetyInformation(kortex_device_manager::GetAllSafetyInformation::Request  &req, kortex_device_manager::GetAllSafetyInformation::Response &res)
{
	Empty input;
	SafetyInformationList output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetAllSafetyInformation();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool KortexDeviceManager::ResetSafetyDefaults(kortex_device_manager::ResetSafetyDefaults::Request  &req, kortex_device_manager::ResetSafetyDefaults::Response &res)
{
	Empty input;
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->ResetSafetyDefaults();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::OnNotificationSafetyTopic(kortex_device_manager::SafetyTopic::Request  &req, kortex_device_manager::SafetyTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		
		std::function< void (Kinova::Api::Common::SafetyNotification) > callback = std::bind(&KortexDeviceManager::cb_SafetyTopic, this, std::placeholders::_1);
		output = m_deviceconfig->OnNotificationSafetyTopic(callback, input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void KortexDeviceManager::cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif)
{
	kortex_device_manager::SafetyNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SafetyTopic.publish(ros_msg);
}

bool KortexDeviceManager::SetModelNumber(kortex_device_manager::SetModelNumber::Request  &req, kortex_device_manager::SetModelNumber::Response &res)
{
	ModelNumber input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetModelNumber(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetPartNumber(kortex_device_manager::SetPartNumber::Request  &req, kortex_device_manager::SetPartNumber::Response &res)
{
	PartNumber input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetPartNumber(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetPartNumberRevision(kortex_device_manager::SetPartNumberRevision::Request  &req, kortex_device_manager::SetPartNumberRevision::Response &res)
{
	PartNumberRevision input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetPartNumberRevision(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetSerialNumber(kortex_device_manager::SetSerialNumber::Request  &req, kortex_device_manager::SetSerialNumber::Response &res)
{
	SerialNumber input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSerialNumber(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool KortexDeviceManager::SetMACAddress(kortex_device_manager::SetMACAddress::Request  &req, kortex_device_manager::SetMACAddress::Response &res)
{
	MACAddress input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetMACAddress(input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}


bool KortexDeviceManager::ReadAllDevices(kortex_device_manager::ReadAllDevices::Request  &req, kortex_device_manager::ReadAllDevices::Response &res)
{
	Empty input;
	DeviceHandles output;
	kortex_device_manager::KortexError result_error;
	
	try
	{
		output = m_devicemanager->ReadAllDevices();
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("KINOVA exception: %d\n", ex.getErrorInfo().getError().error_sub_code());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
