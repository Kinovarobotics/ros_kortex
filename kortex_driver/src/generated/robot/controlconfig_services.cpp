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
#include "kortex_driver/generated/robot/controlconfig_services.h"

ControlConfigRobotServices::ControlConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::ControlConfig::ControlConfigClient* controlconfig, uint32_t device_id, uint32_t timeout_ms): 
	IControlConfigServices(node_handle),
	m_controlconfig(controlconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_ControlConfigurationTopic = m_node_handle.advertise<kortex_driver::ControlConfigurationNotification>("control_configuration_topic", 1000);
	m_is_activated_ControlConfigurationTopic = false;
	m_pub_ControlModeTopic = m_node_handle.advertise<kortex_driver::ControlConfig_ControlModeNotification>("control_mode_topic", 1000);
	m_is_activated_ControlModeTopic = false;

	m_serviceSetDeviceID = m_node_handle.advertiseService("control_config/set_device_id", &ControlConfigRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("control_config/set_api_options", &ControlConfigRobotServices::SetApiOptions, this);

	m_serviceSetGravityVector = m_node_handle.advertiseService("control_config/set_gravity_vector", &ControlConfigRobotServices::SetGravityVector, this);
	m_serviceGetGravityVector = m_node_handle.advertiseService("control_config/get_gravity_vector", &ControlConfigRobotServices::GetGravityVector, this);
	m_serviceSetPayloadInformation = m_node_handle.advertiseService("control_config/set_payload_information", &ControlConfigRobotServices::SetPayloadInformation, this);
	m_serviceGetPayloadInformation = m_node_handle.advertiseService("control_config/get_payload_information", &ControlConfigRobotServices::GetPayloadInformation, this);
	m_serviceSetToolConfiguration = m_node_handle.advertiseService("control_config/set_tool_configuration", &ControlConfigRobotServices::SetToolConfiguration, this);
	m_serviceGetToolConfiguration = m_node_handle.advertiseService("control_config/get_tool_configuration", &ControlConfigRobotServices::GetToolConfiguration, this);
	m_serviceOnNotificationControlConfigurationTopic = m_node_handle.advertiseService("control_config/activate_publishing_of_control_configuration_topic", &ControlConfigRobotServices::OnNotificationControlConfigurationTopic, this);
	m_serviceControlConfig_Unsubscribe = m_node_handle.advertiseService("control_config/unsubscribe", &ControlConfigRobotServices::ControlConfig_Unsubscribe, this);
	m_serviceSetCartesianReferenceFrame = m_node_handle.advertiseService("control_config/set_cartesian_reference_frame", &ControlConfigRobotServices::SetCartesianReferenceFrame, this);
	m_serviceGetCartesianReferenceFrame = m_node_handle.advertiseService("control_config/get_cartesian_reference_frame", &ControlConfigRobotServices::GetCartesianReferenceFrame, this);
	m_serviceControlConfig_GetControlMode = m_node_handle.advertiseService("control_config/get_control_mode", &ControlConfigRobotServices::ControlConfig_GetControlMode, this);
	m_serviceSetJointSpeedSoftLimits = m_node_handle.advertiseService("control_config/set_joint_speed_soft_limits", &ControlConfigRobotServices::SetJointSpeedSoftLimits, this);
	m_serviceSetTwistLinearSoftLimit = m_node_handle.advertiseService("control_config/set_twist_linear_soft_limit", &ControlConfigRobotServices::SetTwistLinearSoftLimit, this);
	m_serviceSetTwistAngularSoftLimit = m_node_handle.advertiseService("control_config/set_twist_angular_soft_limit", &ControlConfigRobotServices::SetTwistAngularSoftLimit, this);
	m_serviceSetJointAccelerationSoftLimits = m_node_handle.advertiseService("control_config/set_joint_acceleration_soft_limits", &ControlConfigRobotServices::SetJointAccelerationSoftLimits, this);
	m_serviceGetKinematicHardLimits = m_node_handle.advertiseService("control_config/get_kinematic_hard_limits", &ControlConfigRobotServices::GetKinematicHardLimits, this);
	m_serviceGetKinematicSoftLimits = m_node_handle.advertiseService("control_config/get_kinematic_soft_limits", &ControlConfigRobotServices::GetKinematicSoftLimits, this);
	m_serviceGetAllKinematicSoftLimits = m_node_handle.advertiseService("control_config/get_all_kinematic_soft_limits", &ControlConfigRobotServices::GetAllKinematicSoftLimits, this);
	m_serviceSetDesiredLinearTwist = m_node_handle.advertiseService("control_config/set_desired_linear_twist", &ControlConfigRobotServices::SetDesiredLinearTwist, this);
	m_serviceSetDesiredAngularTwist = m_node_handle.advertiseService("control_config/set_desired_angular_twist", &ControlConfigRobotServices::SetDesiredAngularTwist, this);
	m_serviceSetDesiredJointSpeeds = m_node_handle.advertiseService("control_config/set_desired_joint_speeds", &ControlConfigRobotServices::SetDesiredJointSpeeds, this);
	m_serviceGetDesiredSpeeds = m_node_handle.advertiseService("control_config/get_desired_speeds", &ControlConfigRobotServices::GetDesiredSpeeds, this);
	m_serviceResetGravityVector = m_node_handle.advertiseService("control_config/reset_gravity_vector", &ControlConfigRobotServices::ResetGravityVector, this);
	m_serviceResetPayloadInformation = m_node_handle.advertiseService("control_config/reset_payload_information", &ControlConfigRobotServices::ResetPayloadInformation, this);
	m_serviceResetToolConfiguration = m_node_handle.advertiseService("control_config/reset_tool_configuration", &ControlConfigRobotServices::ResetToolConfiguration, this);
	m_serviceResetJointSpeedSoftLimits = m_node_handle.advertiseService("control_config/reset_joint_speed_soft_limits", &ControlConfigRobotServices::ResetJointSpeedSoftLimits, this);
	m_serviceResetTwistLinearSoftLimit = m_node_handle.advertiseService("control_config/reset_twist_linear_soft_limit", &ControlConfigRobotServices::ResetTwistLinearSoftLimit, this);
	m_serviceResetTwistAngularSoftLimit = m_node_handle.advertiseService("control_config/reset_twist_angular_soft_limit", &ControlConfigRobotServices::ResetTwistAngularSoftLimit, this);
	m_serviceResetJointAccelerationSoftLimits = m_node_handle.advertiseService("control_config/reset_joint_acceleration_soft_limits", &ControlConfigRobotServices::ResetJointAccelerationSoftLimits, this);
	m_serviceControlConfig_OnNotificationControlModeTopic = m_node_handle.advertiseService("control_config/activate_publishing_of_control_mode_topic", &ControlConfigRobotServices::ControlConfig_OnNotificationControlModeTopic, this);
}

bool ControlConfigRobotServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool ControlConfigRobotServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool ControlConfigRobotServices::SetGravityVector(kortex_driver::SetGravityVector::Request  &req, kortex_driver::SetGravityVector::Response &res)
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

bool ControlConfigRobotServices::GetGravityVector(kortex_driver::GetGravityVector::Request  &req, kortex_driver::GetGravityVector::Response &res)
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

bool ControlConfigRobotServices::SetPayloadInformation(kortex_driver::SetPayloadInformation::Request  &req, kortex_driver::SetPayloadInformation::Response &res)
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

bool ControlConfigRobotServices::GetPayloadInformation(kortex_driver::GetPayloadInformation::Request  &req, kortex_driver::GetPayloadInformation::Response &res)
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

bool ControlConfigRobotServices::SetToolConfiguration(kortex_driver::SetToolConfiguration::Request  &req, kortex_driver::SetToolConfiguration::Response &res)
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

bool ControlConfigRobotServices::GetToolConfiguration(kortex_driver::GetToolConfiguration::Request  &req, kortex_driver::GetToolConfiguration::Response &res)
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

bool ControlConfigRobotServices::OnNotificationControlConfigurationTopic(kortex_driver::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::OnNotificationControlConfigurationTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlConfigurationTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::ControlConfig::ControlConfigurationNotification) > callback = std::bind(&ControlConfigRobotServices::cb_ControlConfigurationTopic, this, std::placeholders::_1);
		output = m_controlconfig->OnNotificationControlConfigurationTopic(callback, input, m_current_device_id);
		m_is_activated_ControlConfigurationTopic = true;
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
void ControlConfigRobotServices::cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif)
{
	kortex_driver::ControlConfigurationNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlConfigurationTopic.publish(ros_msg);
}

bool ControlConfigRobotServices::ControlConfig_Unsubscribe(kortex_driver::ControlConfig_Unsubscribe::Request  &req, kortex_driver::ControlConfig_Unsubscribe::Response &res)
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

bool ControlConfigRobotServices::SetCartesianReferenceFrame(kortex_driver::SetCartesianReferenceFrame::Request  &req, kortex_driver::SetCartesianReferenceFrame::Response &res)
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

bool ControlConfigRobotServices::GetCartesianReferenceFrame(kortex_driver::GetCartesianReferenceFrame::Request  &req, kortex_driver::GetCartesianReferenceFrame::Response &res)
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

bool ControlConfigRobotServices::ControlConfig_GetControlMode(kortex_driver::ControlConfig_GetControlMode::Request  &req, kortex_driver::ControlConfig_GetControlMode::Response &res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetControlMode(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetJointSpeedSoftLimits(kortex_driver::SetJointSpeedSoftLimits::Request  &req, kortex_driver::SetJointSpeedSoftLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::JointSpeedSoftLimits input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetJointSpeedSoftLimits(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetTwistLinearSoftLimit(kortex_driver::SetTwistLinearSoftLimit::Request  &req, kortex_driver::SetTwistLinearSoftLimit::Response &res)
{
	
	Kinova::Api::ControlConfig::TwistLinearSoftLimit input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetTwistLinearSoftLimit(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetTwistAngularSoftLimit(kortex_driver::SetTwistAngularSoftLimit::Request  &req, kortex_driver::SetTwistAngularSoftLimit::Response &res)
{
	
	Kinova::Api::ControlConfig::TwistAngularSoftLimit input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetTwistAngularSoftLimit(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetJointAccelerationSoftLimits(kortex_driver::SetJointAccelerationSoftLimits::Request  &req, kortex_driver::SetJointAccelerationSoftLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::JointAccelerationSoftLimits input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetJointAccelerationSoftLimits(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::GetKinematicHardLimits(kortex_driver::GetKinematicHardLimits::Request  &req, kortex_driver::GetKinematicHardLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::KinematicLimits output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetKinematicHardLimits(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::GetKinematicSoftLimits(kortex_driver::GetKinematicSoftLimits::Request  &req, kortex_driver::GetKinematicSoftLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	Kinova::Api::ControlConfig::KinematicLimits output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetKinematicSoftLimits(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::GetAllKinematicSoftLimits(kortex_driver::GetAllKinematicSoftLimits::Request  &req, kortex_driver::GetAllKinematicSoftLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::KinematicLimitsList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetAllKinematicSoftLimits(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetDesiredLinearTwist(kortex_driver::SetDesiredLinearTwist::Request  &req, kortex_driver::SetDesiredLinearTwist::Response &res)
{
	
	Kinova::Api::ControlConfig::LinearTwist input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetDesiredLinearTwist(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetDesiredAngularTwist(kortex_driver::SetDesiredAngularTwist::Request  &req, kortex_driver::SetDesiredAngularTwist::Response &res)
{
	
	Kinova::Api::ControlConfig::AngularTwist input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetDesiredAngularTwist(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::SetDesiredJointSpeeds(kortex_driver::SetDesiredJointSpeeds::Request  &req, kortex_driver::SetDesiredJointSpeeds::Response &res)
{
	
	Kinova::Api::ControlConfig::JointSpeeds input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_controlconfig->SetDesiredJointSpeeds(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::GetDesiredSpeeds(kortex_driver::GetDesiredSpeeds::Request  &req, kortex_driver::GetDesiredSpeeds::Response &res)
{
	
	Kinova::Api::ControlConfig::DesiredSpeeds output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetDesiredSpeeds(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetGravityVector(kortex_driver::ResetGravityVector::Request  &req, kortex_driver::ResetGravityVector::Response &res)
{
	
	Kinova::Api::ControlConfig::GravityVector output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetGravityVector(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetPayloadInformation(kortex_driver::ResetPayloadInformation::Request  &req, kortex_driver::ResetPayloadInformation::Response &res)
{
	
	Kinova::Api::ControlConfig::PayloadInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetPayloadInformation(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetToolConfiguration(kortex_driver::ResetToolConfiguration::Request  &req, kortex_driver::ResetToolConfiguration::Response &res)
{
	
	Kinova::Api::ControlConfig::ToolConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetToolConfiguration(m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetJointSpeedSoftLimits(kortex_driver::ResetJointSpeedSoftLimits::Request  &req, kortex_driver::ResetJointSpeedSoftLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	Kinova::Api::ControlConfig::JointSpeedSoftLimits output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetJointSpeedSoftLimits(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetTwistLinearSoftLimit(kortex_driver::ResetTwistLinearSoftLimit::Request  &req, kortex_driver::ResetTwistLinearSoftLimit::Response &res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	Kinova::Api::ControlConfig::TwistLinearSoftLimit output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetTwistLinearSoftLimit(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetTwistAngularSoftLimit(kortex_driver::ResetTwistAngularSoftLimit::Request  &req, kortex_driver::ResetTwistAngularSoftLimit::Response &res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	Kinova::Api::ControlConfig::TwistAngularSoftLimit output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetTwistAngularSoftLimit(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ResetJointAccelerationSoftLimits(kortex_driver::ResetJointAccelerationSoftLimits::Request  &req, kortex_driver::ResetJointAccelerationSoftLimits::Response &res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	Kinova::Api::ControlConfig::JointAccelerationSoftLimits output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetJointAccelerationSoftLimits(input, m_current_device_id, m_api_options);
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

bool ControlConfigRobotServices::ControlConfig_OnNotificationControlModeTopic(kortex_driver::ControlConfig_OnNotificationControlModeTopic::Request  &req, kortex_driver::ControlConfig_OnNotificationControlModeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::ControlConfig::ControlModeNotification) > callback = std::bind(&ControlConfigRobotServices::cb_ControlModeTopic, this, std::placeholders::_1);
		output = m_controlconfig->OnNotificationControlModeTopic(callback, input, m_current_device_id);
		m_is_activated_ControlModeTopic = true;
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
void ControlConfigRobotServices::cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif)
{
	kortex_driver::ControlConfig_ControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic.publish(ros_msg);
}
