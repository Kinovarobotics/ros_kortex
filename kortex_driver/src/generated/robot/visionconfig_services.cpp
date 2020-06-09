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
#include "kortex_driver/generated/robot/visionconfig_services.h"

VisionConfigRobotServices::VisionConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::VisionConfig::VisionConfigClient* visionconfig, uint32_t device_id, uint32_t timeout_ms): 
	IVisionConfigServices(node_handle),
	m_visionconfig(visionconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_VisionTopic = m_node_handle.advertise<kortex_driver::VisionNotification>("vision_topic", 1000);
	m_is_activated_VisionTopic = false;

	m_serviceSetDeviceID = m_node_handle.advertiseService("vision_config/set_device_id", &VisionConfigRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("vision_config/set_api_options", &VisionConfigRobotServices::SetApiOptions, this);

	m_serviceSetSensorSettings = m_node_handle.advertiseService("vision_config/set_sensor_settings", &VisionConfigRobotServices::SetSensorSettings, this);
	m_serviceGetSensorSettings = m_node_handle.advertiseService("vision_config/get_sensor_settings", &VisionConfigRobotServices::GetSensorSettings, this);
	m_serviceGetOptionValue = m_node_handle.advertiseService("vision_config/get_option_value", &VisionConfigRobotServices::GetOptionValue, this);
	m_serviceSetOptionValue = m_node_handle.advertiseService("vision_config/set_option_value", &VisionConfigRobotServices::SetOptionValue, this);
	m_serviceGetOptionInformation = m_node_handle.advertiseService("vision_config/get_option_information", &VisionConfigRobotServices::GetOptionInformation, this);
	m_serviceOnNotificationVisionTopic = m_node_handle.advertiseService("vision_config/activate_publishing_of_vision_topic", &VisionConfigRobotServices::OnNotificationVisionTopic, this);
	m_serviceDoSensorFocusAction = m_node_handle.advertiseService("vision_config/do_sensor_focus_action", &VisionConfigRobotServices::DoSensorFocusAction, this);
	m_serviceGetIntrinsicParameters = m_node_handle.advertiseService("vision_config/get_intrinsic_parameters", &VisionConfigRobotServices::GetIntrinsicParameters, this);
	m_serviceGetIntrinsicParametersProfile = m_node_handle.advertiseService("vision_config/get_intrinsic_parameters_profile", &VisionConfigRobotServices::GetIntrinsicParametersProfile, this);
	m_serviceSetIntrinsicParameters = m_node_handle.advertiseService("vision_config/set_intrinsic_parameters", &VisionConfigRobotServices::SetIntrinsicParameters, this);
	m_serviceGetExtrinsicParameters = m_node_handle.advertiseService("vision_config/get_extrinsic_parameters", &VisionConfigRobotServices::GetExtrinsicParameters, this);
	m_serviceSetExtrinsicParameters = m_node_handle.advertiseService("vision_config/set_extrinsic_parameters", &VisionConfigRobotServices::SetExtrinsicParameters, this);
}

bool VisionConfigRobotServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool VisionConfigRobotServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool VisionConfigRobotServices::SetSensorSettings(kortex_driver::SetSensorSettings::Request  &req, kortex_driver::SetSensorSettings::Response &res)
{
	
	Kinova::Api::VisionConfig::SensorSettings input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->SetSensorSettings(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetSensorSettings(kortex_driver::GetSensorSettings::Request  &req, kortex_driver::GetSensorSettings::Response &res)
{
	
	Kinova::Api::VisionConfig::SensorIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::VisionConfig::SensorSettings output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetSensorSettings(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetOptionValue(kortex_driver::GetOptionValue::Request  &req, kortex_driver::GetOptionValue::Response &res)
{
	
	Kinova::Api::VisionConfig::OptionIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::VisionConfig::OptionValue output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetOptionValue(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::SetOptionValue(kortex_driver::SetOptionValue::Request  &req, kortex_driver::SetOptionValue::Response &res)
{
	
	Kinova::Api::VisionConfig::OptionValue input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->SetOptionValue(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetOptionInformation(kortex_driver::GetOptionInformation::Request  &req, kortex_driver::GetOptionInformation::Response &res)
{
	
	Kinova::Api::VisionConfig::OptionIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::VisionConfig::OptionInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetOptionInformation(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::OnNotificationVisionTopic(kortex_driver::OnNotificationVisionTopic::Request  &req, kortex_driver::OnNotificationVisionTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_VisionTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::VisionConfig::VisionNotification) > callback = std::bind(&VisionConfigRobotServices::cb_VisionTopic, this, std::placeholders::_1);
		output = m_visionconfig->OnNotificationVisionTopic(callback, input, m_current_device_id);
		m_is_activated_VisionTopic = true;
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
void VisionConfigRobotServices::cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif)
{
	kortex_driver::VisionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_VisionTopic.publish(ros_msg);
}

bool VisionConfigRobotServices::DoSensorFocusAction(kortex_driver::DoSensorFocusAction::Request  &req, kortex_driver::DoSensorFocusAction::Response &res)
{
	
	Kinova::Api::VisionConfig::SensorFocusAction input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->DoSensorFocusAction(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetIntrinsicParameters(kortex_driver::GetIntrinsicParameters::Request  &req, kortex_driver::GetIntrinsicParameters::Response &res)
{
	
	Kinova::Api::VisionConfig::SensorIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::VisionConfig::IntrinsicParameters output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetIntrinsicParameters(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetIntrinsicParametersProfile(kortex_driver::GetIntrinsicParametersProfile::Request  &req, kortex_driver::GetIntrinsicParametersProfile::Response &res)
{
	
	Kinova::Api::VisionConfig::IntrinsicProfileIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::VisionConfig::IntrinsicParameters output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetIntrinsicParametersProfile(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::SetIntrinsicParameters(kortex_driver::SetIntrinsicParameters::Request  &req, kortex_driver::SetIntrinsicParameters::Response &res)
{
	
	Kinova::Api::VisionConfig::IntrinsicParameters input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->SetIntrinsicParameters(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetExtrinsicParameters(kortex_driver::GetExtrinsicParameters::Request  &req, kortex_driver::GetExtrinsicParameters::Response &res)
{
	
	Kinova::Api::VisionConfig::ExtrinsicParameters output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetExtrinsicParameters(m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::SetExtrinsicParameters(kortex_driver::SetExtrinsicParameters::Request  &req, kortex_driver::SetExtrinsicParameters::Response &res)
{
	
	Kinova::Api::VisionConfig::ExtrinsicParameters input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_visionconfig->SetExtrinsicParameters(input, m_current_device_id, m_api_options);
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
