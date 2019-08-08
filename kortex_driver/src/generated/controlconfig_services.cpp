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
#include "kortex_driver/generated/controlconfig_services.h"

ControlConfigServices::ControlConfigServices(ros::NodeHandle& n, Kinova::Api::ControlConfig::ControlConfigClient* controlconfig, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_controlconfig(controlconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_ControlConfigurationTopic = m_n.advertise<kortex_driver::ControlConfigurationNotification>("control_configuration_topic", 1000);

	m_serviceSetDeviceID = n.advertiseService("control_config/set_device_id", &ControlConfigServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("control_config/set_api_options", &ControlConfigServices::SetApiOptions, this);

	m_serviceSetGravityVector = m_n.advertiseService("control_config/set_gravity_vector", &ControlConfigServices::SetGravityVector, this);
	m_serviceGetGravityVector = m_n.advertiseService("control_config/get_gravity_vector", &ControlConfigServices::GetGravityVector, this);
	m_serviceSetPayloadInformation = m_n.advertiseService("control_config/set_payload_information", &ControlConfigServices::SetPayloadInformation, this);
	m_serviceGetPayloadInformation = m_n.advertiseService("control_config/get_payload_information", &ControlConfigServices::GetPayloadInformation, this);
	m_serviceSetToolConfiguration = m_n.advertiseService("control_config/set_tool_configuration", &ControlConfigServices::SetToolConfiguration, this);
	m_serviceGetToolConfiguration = m_n.advertiseService("control_config/get_tool_configuration", &ControlConfigServices::GetToolConfiguration, this);
	m_serviceOnNotificationControlConfigurationTopic = m_n.advertiseService("control_config/activate_publishing_of_control_configuration_topic", &ControlConfigServices::OnNotificationControlConfigurationTopic, this);
	m_serviceControlConfig_Unsubscribe = m_n.advertiseService("control_config/unsubscribe", &ControlConfigServices::ControlConfig_Unsubscribe, this);
	m_serviceSetCartesianReferenceFrame = m_n.advertiseService("control_config/set_cartesian_reference_frame", &ControlConfigServices::SetCartesianReferenceFrame, this);
	m_serviceGetCartesianReferenceFrame = m_n.advertiseService("control_config/get_cartesian_reference_frame", &ControlConfigServices::GetCartesianReferenceFrame, this);
}

bool ControlConfigServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool ControlConfigServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool ControlConfigServices::SetGravityVector(kortex_driver::SetGravityVector::Request  &req, kortex_driver::SetGravityVector::Response &res)
{
	Kinova::Api::ControlConfig::GravityVector input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetGravityVector(input, m_current_device_id, m_api_options);
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

bool ControlConfigServices::GetGravityVector(kortex_driver::GetGravityVector::Request  &req, kortex_driver::GetGravityVector::Response &res)
{
	Kinova::Api::ControlConfig::GravityVector output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetGravityVector(m_current_device_id, m_api_options);
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

bool ControlConfigServices::SetPayloadInformation(kortex_driver::SetPayloadInformation::Request  &req, kortex_driver::SetPayloadInformation::Response &res)
{
	Kinova::Api::ControlConfig::PayloadInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetPayloadInformation(input, m_current_device_id, m_api_options);
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

bool ControlConfigServices::GetPayloadInformation(kortex_driver::GetPayloadInformation::Request  &req, kortex_driver::GetPayloadInformation::Response &res)
{
	Kinova::Api::ControlConfig::PayloadInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetPayloadInformation(m_current_device_id, m_api_options);
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

bool ControlConfigServices::SetToolConfiguration(kortex_driver::SetToolConfiguration::Request  &req, kortex_driver::SetToolConfiguration::Response &res)
{
	Kinova::Api::ControlConfig::ToolConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetToolConfiguration(input, m_current_device_id, m_api_options);
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

bool ControlConfigServices::GetToolConfiguration(kortex_driver::GetToolConfiguration::Request  &req, kortex_driver::GetToolConfiguration::Response &res)
{
	Kinova::Api::ControlConfig::ToolConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetToolConfiguration(m_current_device_id, m_api_options);
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

bool ControlConfigServices::OnNotificationControlConfigurationTopic(kortex_driver::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::OnNotificationControlConfigurationTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::ControlConfig::ControlConfigurationNotification) > callback = std::bind(&ControlConfigServices::cb_ControlConfigurationTopic, this, std::placeholders::_1);
		output = m_controlconfig->OnNotificationControlConfigurationTopic(callback, input, m_current_device_id);
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
void ControlConfigServices::cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif)
{
	kortex_driver::ControlConfigurationNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlConfigurationTopic.publish(ros_msg);
}

bool ControlConfigServices::ControlConfig_Unsubscribe(kortex_driver::ControlConfig_Unsubscribe::Request  &req, kortex_driver::ControlConfig_Unsubscribe::Response &res)
{
	Kinova::Api::Common::NotificationHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->Unsubscribe(input, m_current_device_id, m_api_options);
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

bool ControlConfigServices::SetCartesianReferenceFrame(kortex_driver::SetCartesianReferenceFrame::Request  &req, kortex_driver::SetCartesianReferenceFrame::Response &res)
{
	Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetCartesianReferenceFrame(input, m_current_device_id, m_api_options);
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

bool ControlConfigServices::GetCartesianReferenceFrame(kortex_driver::GetCartesianReferenceFrame::Request  &req, kortex_driver::GetCartesianReferenceFrame::Response &res)
{
	Kinova::Api::ControlConfig::CartesianReferenceFrameInfo output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetCartesianReferenceFrame(m_current_device_id, m_api_options);
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
