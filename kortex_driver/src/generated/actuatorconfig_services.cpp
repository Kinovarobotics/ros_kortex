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
#include "kortex_driver/generated/actuatorconfig_services.h"

ActuatorConfigServices::ActuatorConfigServices(ros::NodeHandle& n, Kinova::Api::ActuatorConfig::ActuatorConfigClient* actuatorconfig, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_actuatorconfig(actuatorconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = n.advertiseService("actuator_config/set_device_id", &ActuatorConfigServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("actuator_config/set_api_options", &ActuatorConfigServices::SetApiOptions, this);

	m_serviceGetAxisOffsets = m_n.advertiseService("actuator_config/get_axis_offsets", &ActuatorConfigServices::GetAxisOffsets, this);
	m_serviceSetAxisOffsets = m_n.advertiseService("actuator_config/set_axis_offsets", &ActuatorConfigServices::SetAxisOffsets, this);
	m_serviceSetTorqueOffset = m_n.advertiseService("actuator_config/set_torque_offset", &ActuatorConfigServices::SetTorqueOffset, this);
	m_serviceActuatorConfig_GetControlMode = m_n.advertiseService("actuator_config/get_control_mode", &ActuatorConfigServices::ActuatorConfig_GetControlMode, this);
	m_serviceSetControlMode = m_n.advertiseService("actuator_config/set_control_mode", &ActuatorConfigServices::SetControlMode, this);
	m_serviceGetActivatedControlLoop = m_n.advertiseService("actuator_config/get_activated_control_loop", &ActuatorConfigServices::GetActivatedControlLoop, this);
	m_serviceSetActivatedControlLoop = m_n.advertiseService("actuator_config/set_activated_control_loop", &ActuatorConfigServices::SetActivatedControlLoop, this);
	m_serviceGetControlLoopParameters = m_n.advertiseService("actuator_config/get_control_loop_parameters", &ActuatorConfigServices::GetControlLoopParameters, this);
	m_serviceSetControlLoopParameters = m_n.advertiseService("actuator_config/set_control_loop_parameters", &ActuatorConfigServices::SetControlLoopParameters, this);
	m_serviceSelectCustomData = m_n.advertiseService("actuator_config/select_custom_data", &ActuatorConfigServices::SelectCustomData, this);
	m_serviceGetSelectedCustomData = m_n.advertiseService("actuator_config/get_selected_custom_data", &ActuatorConfigServices::GetSelectedCustomData, this);
	m_serviceSetCommandMode = m_n.advertiseService("actuator_config/set_command_mode", &ActuatorConfigServices::SetCommandMode, this);
	m_serviceActuatorConfig_ClearFaults = m_n.advertiseService("actuator_config/clear_faults", &ActuatorConfigServices::ActuatorConfig_ClearFaults, this);
	m_serviceSetServoing = m_n.advertiseService("actuator_config/set_servoing", &ActuatorConfigServices::SetServoing, this);
	m_serviceMoveToPosition = m_n.advertiseService("actuator_config/move_to_position", &ActuatorConfigServices::MoveToPosition, this);
	m_serviceGetCommandMode = m_n.advertiseService("actuator_config/get_command_mode", &ActuatorConfigServices::GetCommandMode, this);
	m_serviceGetServoing = m_n.advertiseService("actuator_config/get_servoing", &ActuatorConfigServices::GetServoing, this);
	m_serviceGetTorqueOffset = m_n.advertiseService("actuator_config/get_torque_offset", &ActuatorConfigServices::GetTorqueOffset, this);
}

bool ActuatorConfigServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool ActuatorConfigServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool ActuatorConfigServices::GetAxisOffsets(kortex_driver::GetAxisOffsets::Request  &req, kortex_driver::GetAxisOffsets::Response &res)
{
	Kinova::Api::ActuatorConfig::AxisOffsets output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetAxisOffsets(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetAxisOffsets(kortex_driver::SetAxisOffsets::Request  &req, kortex_driver::SetAxisOffsets::Response &res)
{
	Kinova::Api::ActuatorConfig::AxisPosition input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetAxisOffsets(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetTorqueOffset(kortex_driver::SetTorqueOffset::Request  &req, kortex_driver::SetTorqueOffset::Response &res)
{
	Kinova::Api::ActuatorConfig::TorqueOffset input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetTorqueOffset(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::ActuatorConfig_GetControlMode(kortex_driver::ActuatorConfig_GetControlMode::Request  &req, kortex_driver::ActuatorConfig_GetControlMode::Response &res)
{
	Kinova::Api::ActuatorConfig::ControlModeInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetControlMode(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetControlMode(kortex_driver::SetControlMode::Request  &req, kortex_driver::SetControlMode::Response &res)
{
	Kinova::Api::ActuatorConfig::ControlModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetControlMode(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::GetActivatedControlLoop(kortex_driver::GetActivatedControlLoop::Request  &req, kortex_driver::GetActivatedControlLoop::Response &res)
{
	Kinova::Api::ActuatorConfig::ControlLoop output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetActivatedControlLoop(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetActivatedControlLoop(kortex_driver::SetActivatedControlLoop::Request  &req, kortex_driver::SetActivatedControlLoop::Response &res)
{
	Kinova::Api::ActuatorConfig::ControlLoop input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetActivatedControlLoop(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::GetControlLoopParameters(kortex_driver::GetControlLoopParameters::Request  &req, kortex_driver::GetControlLoopParameters::Response &res)
{
	Kinova::Api::ActuatorConfig::LoopSelection input;
	ToProtoData(req.input, &input);
	Kinova::Api::ActuatorConfig::ControlLoopParameters output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetControlLoopParameters(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetControlLoopParameters(kortex_driver::SetControlLoopParameters::Request  &req, kortex_driver::SetControlLoopParameters::Response &res)
{
	Kinova::Api::ActuatorConfig::ControlLoopParameters input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetControlLoopParameters(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SelectCustomData(kortex_driver::SelectCustomData::Request  &req, kortex_driver::SelectCustomData::Response &res)
{
	Kinova::Api::ActuatorConfig::CustomDataSelection input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SelectCustomData(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::GetSelectedCustomData(kortex_driver::GetSelectedCustomData::Request  &req, kortex_driver::GetSelectedCustomData::Response &res)
{
	Kinova::Api::ActuatorConfig::CustomDataSelection output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetSelectedCustomData(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetCommandMode(kortex_driver::SetCommandMode::Request  &req, kortex_driver::SetCommandMode::Response &res)
{
	Kinova::Api::ActuatorConfig::CommandModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetCommandMode(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::ActuatorConfig_ClearFaults(kortex_driver::ActuatorConfig_ClearFaults::Request  &req, kortex_driver::ActuatorConfig_ClearFaults::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->ClearFaults(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::SetServoing(kortex_driver::SetServoing::Request  &req, kortex_driver::SetServoing::Response &res)
{
	Kinova::Api::ActuatorConfig::Servoing input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->SetServoing(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::MoveToPosition(kortex_driver::MoveToPosition::Request  &req, kortex_driver::MoveToPosition::Response &res)
{
	Kinova::Api::ActuatorConfig::PositionCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_actuatorconfig->MoveToPosition(input, m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::GetCommandMode(kortex_driver::GetCommandMode::Request  &req, kortex_driver::GetCommandMode::Response &res)
{
	Kinova::Api::ActuatorConfig::CommandModeInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetCommandMode(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::GetServoing(kortex_driver::GetServoing::Request  &req, kortex_driver::GetServoing::Response &res)
{
	Kinova::Api::ActuatorConfig::Servoing output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetServoing(m_current_device_id, m_api_options);
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

bool ActuatorConfigServices::GetTorqueOffset(kortex_driver::GetTorqueOffset::Request  &req, kortex_driver::GetTorqueOffset::Response &res)
{
	Kinova::Api::ActuatorConfig::TorqueOffset output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_actuatorconfig->GetTorqueOffset(m_current_device_id, m_api_options);
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
