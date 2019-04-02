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
#include "visionconfig_ros_converter.h"
#include "visionconfig_proto_converter.h"
VisionConfig_Services::VisionConfig_Services(char* ip, ros::NodeHandle& n, uint32_t device_id) : m_n(n)
{
	m_transport = new TransportClientUdp();
	m_transport->connect(ip, 10000);

	m_router = new RouterClient(m_transport, [](KError err) { cout << "_________ callback error _________" << err.toString(); });
	m_CurrentDeviceID = device_id;
	m_apiOptions.timeout_ms = 3000;

	m_visionconfig = new VisionConfig::VisionConfigClient(m_router);
	//If the Device ID is different than 0, it means that we are using the feature DEVICE ROUTING.
	if(m_CurrentDeviceID != 0)
	{
		m_SessionManager = new SessionManager(m_router);
		auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
		
		createSessionInfo.set_username("admin");
		createSessionInfo.set_password("admin");
		createSessionInfo.set_session_inactivity_timeout(35000);

		m_SessionManager->CreateSession(createSessionInfo);
	}
	
	m_pub_Error = m_n.advertise<kortex_vision_config_driver::KortexError>("KortexError", 1000);
	m_pub_VisionTopic = m_n.advertise<kortex_vision_config_driver::VisionNotification>("VisionTopic", 1000);std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

bool VisionConfig_Services::SetDeviceID(kortex_vision_config_driver::SetDeviceID::Request  &req, kortex_vision_config_driver::SetDeviceID::Response &res)
{
	if(m_CurrentDeviceID == 0)
	{
		auto sessionManager = new SessionManager(m_router);
		auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
		
		createSessionInfo.set_username("admin");
		createSessionInfo.set_password("admin");
		createSessionInfo.set_session_inactivity_timeout(35000);

		sessionManager->CreateSession(createSessionInfo);
	}
	
	m_CurrentDeviceID = req.device_id;
}

bool VisionConfig_Services::SetApiOptions(kortex_vision_config_driver::SetApiOptions::Request  &req, kortex_vision_config_driver::SetApiOptions::Response &res)
{
	m_apiOptions.timeout_ms = req.input.timeout_ms;

	return true;
}



bool VisionConfig_Services::SetSensorSettings(kortex_vision_config_driver::SetSensorSettings::Request  &req, kortex_vision_config_driver::SetSensorSettings::Response &res)
{
	SensorSettings input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->SetSensorSettings(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool VisionConfig_Services::GetSensorSettings(kortex_vision_config_driver::GetSensorSettings::Request  &req, kortex_vision_config_driver::GetSensorSettings::Response &res)
{
	SensorIdentifier input;
	ToProtoData(req.input, &input);
	SensorSettings output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetSensorSettings(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool VisionConfig_Services::GetOptionValue(kortex_vision_config_driver::GetOptionValue::Request  &req, kortex_vision_config_driver::GetOptionValue::Response &res)
{
	OptionIdentifier input;
	ToProtoData(req.input, &input);
	OptionValue output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetOptionValue(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool VisionConfig_Services::SetOptionValue(kortex_vision_config_driver::SetOptionValue::Request  &req, kortex_vision_config_driver::SetOptionValue::Response &res)
{
	OptionValue input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->SetOptionValue(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool VisionConfig_Services::GetOptionInformation(kortex_vision_config_driver::GetOptionInformation::Request  &req, kortex_vision_config_driver::GetOptionInformation::Response &res)
{
	OptionIdentifier input;
	ToProtoData(req.input, &input);
	OptionInformation output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetOptionInformation(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool VisionConfig_Services::OnNotificationVisionTopic(kortex_vision_config_driver::OnNotificationVisionTopic::Request  &req, kortex_vision_config_driver::OnNotificationVisionTopic::Response &res)
{
	NotificationOptions input;
	ToProtoData(req.input, &input);
	NotificationHandle output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		std::function< void (VisionConfig::VisionNotification) > callback = std::bind(&VisionConfig_Services::cb_VisionTopic, this, std::placeholders::_1);
		output = m_visionconfig->OnNotificationVisionTopic(callback, input);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void VisionConfig_Services::cb_VisionTopic(VisionConfig::VisionNotification notif)
{
	kortex_vision_config_driver::VisionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_VisionTopic.publish(ros_msg);
}

bool VisionConfig_Services::DoSensorFocusAction(kortex_vision_config_driver::DoSensorFocusAction::Request  &req, kortex_vision_config_driver::DoSensorFocusAction::Response &res)
{
	SensorFocusAction input;
	ToProtoData(req.input, &input);
	Empty output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->DoSensorFocusAction(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	return true;
}

bool VisionConfig_Services::GetIntrinsicParameters(kortex_vision_config_driver::GetIntrinsicParameters::Request  &req, kortex_vision_config_driver::GetIntrinsicParameters::Response &res)
{
	SensorIdentifier input;
	ToProtoData(req.input, &input);
	IntrinsicParameters output;
	kortex_vision_config_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetIntrinsicParameters(input, m_CurrentDeviceID, m_apiOptions);
	}
	catch (KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.what();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
