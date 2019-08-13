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
#include "kortex_driver/generated/visionconfig_services.h"

VisionConfigServices::VisionConfigServices(ros::NodeHandle& n, Kinova::Api::VisionConfig::VisionConfigClient* visionconfig, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_visionconfig(visionconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_VisionTopic = m_n.advertise<kortex_driver::VisionNotification>("vision_topic", 1000);

	m_serviceSetDeviceID = n.advertiseService("vision_config/set_device_id", &VisionConfigServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("vision_config/set_api_options", &VisionConfigServices::SetApiOptions, this);

	m_serviceSetSensorSettings = m_n.advertiseService("vision_config/set_sensor_settings", &VisionConfigServices::SetSensorSettings, this);
	m_serviceGetSensorSettings = m_n.advertiseService("vision_config/get_sensor_settings", &VisionConfigServices::GetSensorSettings, this);
	m_serviceGetOptionValue = m_n.advertiseService("vision_config/get_option_value", &VisionConfigServices::GetOptionValue, this);
	m_serviceSetOptionValue = m_n.advertiseService("vision_config/set_option_value", &VisionConfigServices::SetOptionValue, this);
	m_serviceGetOptionInformation = m_n.advertiseService("vision_config/get_option_information", &VisionConfigServices::GetOptionInformation, this);
	m_serviceOnNotificationVisionTopic = m_n.advertiseService("vision_config/activate_publishing_of_vision_topic", &VisionConfigServices::OnNotificationVisionTopic, this);
	m_serviceDoSensorFocusAction = m_n.advertiseService("vision_config/do_sensor_focus_action", &VisionConfigServices::DoSensorFocusAction, this);
	m_serviceGetIntrinsicParameters = m_n.advertiseService("vision_config/get_intrinsic_parameters", &VisionConfigServices::GetIntrinsicParameters, this);
	m_serviceGetIntrinsicParametersProfile = m_n.advertiseService("vision_config/get_intrinsic_parameters_profile", &VisionConfigServices::GetIntrinsicParametersProfile, this);
	m_serviceSetIntrinsicParameters = m_n.advertiseService("vision_config/set_intrinsic_parameters", &VisionConfigServices::SetIntrinsicParameters, this);
	m_serviceGetExtrinsicParameters = m_n.advertiseService("vision_config/get_extrinsic_parameters", &VisionConfigServices::GetExtrinsicParameters, this);
	m_serviceSetExtrinsicParameters = m_n.advertiseService("vision_config/set_extrinsic_parameters", &VisionConfigServices::SetExtrinsicParameters, this);
}

bool VisionConfigServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool VisionConfigServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool VisionConfigServices::SetSensorSettings(kortex_driver::SetSensorSettings::Request  &req, kortex_driver::SetSensorSettings::Response &res)
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

bool VisionConfigServices::GetSensorSettings(kortex_driver::GetSensorSettings::Request  &req, kortex_driver::GetSensorSettings::Response &res)
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

bool VisionConfigServices::GetOptionValue(kortex_driver::GetOptionValue::Request  &req, kortex_driver::GetOptionValue::Response &res)
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

bool VisionConfigServices::SetOptionValue(kortex_driver::SetOptionValue::Request  &req, kortex_driver::SetOptionValue::Response &res)
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

bool VisionConfigServices::GetOptionInformation(kortex_driver::GetOptionInformation::Request  &req, kortex_driver::GetOptionInformation::Response &res)
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

bool VisionConfigServices::OnNotificationVisionTopic(kortex_driver::OnNotificationVisionTopic::Request  &req, kortex_driver::OnNotificationVisionTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::VisionConfig::VisionNotification) > callback = std::bind(&VisionConfigServices::cb_VisionTopic, this, std::placeholders::_1);
		output = m_visionconfig->OnNotificationVisionTopic(callback, input, m_current_device_id);
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
void VisionConfigServices::cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif)
{
	kortex_driver::VisionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_VisionTopic.publish(ros_msg);
}

bool VisionConfigServices::DoSensorFocusAction(kortex_driver::DoSensorFocusAction::Request  &req, kortex_driver::DoSensorFocusAction::Response &res)
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

bool VisionConfigServices::GetIntrinsicParameters(kortex_driver::GetIntrinsicParameters::Request  &req, kortex_driver::GetIntrinsicParameters::Response &res)
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

bool VisionConfigServices::GetIntrinsicParametersProfile(kortex_driver::GetIntrinsicParametersProfile::Request  &req, kortex_driver::GetIntrinsicParametersProfile::Response &res)
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

bool VisionConfigServices::SetIntrinsicParameters(kortex_driver::SetIntrinsicParameters::Request  &req, kortex_driver::SetIntrinsicParameters::Response &res)
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

bool VisionConfigServices::GetExtrinsicParameters(kortex_driver::GetExtrinsicParameters::Request  &req, kortex_driver::GetExtrinsicParameters::Response &res)
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

bool VisionConfigServices::SetExtrinsicParameters(kortex_driver::SetExtrinsicParameters::Request  &req, kortex_driver::SetExtrinsicParameters::Response &res)
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
