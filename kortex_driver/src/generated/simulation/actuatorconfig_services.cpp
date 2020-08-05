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
#include "kortex_driver/generated/simulation/actuatorconfig_services.h"

ActuatorConfigSimulationServices::ActuatorConfigSimulationServices(ros::NodeHandle& node_handle): 
	IActuatorConfigServices(node_handle)
{
	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle.advertiseService("actuator_config/set_device_id", &ActuatorConfigSimulationServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("actuator_config/set_api_options", &ActuatorConfigSimulationServices::SetApiOptions, this);

	m_serviceGetAxisOffsets = m_node_handle.advertiseService("actuator_config/get_axis_offsets", &ActuatorConfigSimulationServices::GetAxisOffsets, this);
	m_serviceSetAxisOffsets = m_node_handle.advertiseService("actuator_config/set_axis_offsets", &ActuatorConfigSimulationServices::SetAxisOffsets, this);
	m_serviceSetTorqueOffset = m_node_handle.advertiseService("actuator_config/set_torque_offset", &ActuatorConfigSimulationServices::SetTorqueOffset, this);
	m_serviceActuatorConfig_GetControlMode = m_node_handle.advertiseService("actuator_config/get_control_mode", &ActuatorConfigSimulationServices::ActuatorConfig_GetControlMode, this);
	m_serviceSetControlMode = m_node_handle.advertiseService("actuator_config/set_control_mode", &ActuatorConfigSimulationServices::SetControlMode, this);
	m_serviceGetActivatedControlLoop = m_node_handle.advertiseService("actuator_config/get_activated_control_loop", &ActuatorConfigSimulationServices::GetActivatedControlLoop, this);
	m_serviceSetActivatedControlLoop = m_node_handle.advertiseService("actuator_config/set_activated_control_loop", &ActuatorConfigSimulationServices::SetActivatedControlLoop, this);
	m_serviceGetControlLoopParameters = m_node_handle.advertiseService("actuator_config/get_control_loop_parameters", &ActuatorConfigSimulationServices::GetControlLoopParameters, this);
	m_serviceSetControlLoopParameters = m_node_handle.advertiseService("actuator_config/set_control_loop_parameters", &ActuatorConfigSimulationServices::SetControlLoopParameters, this);
	m_serviceSelectCustomData = m_node_handle.advertiseService("actuator_config/select_custom_data", &ActuatorConfigSimulationServices::SelectCustomData, this);
	m_serviceGetSelectedCustomData = m_node_handle.advertiseService("actuator_config/get_selected_custom_data", &ActuatorConfigSimulationServices::GetSelectedCustomData, this);
	m_serviceSetCommandMode = m_node_handle.advertiseService("actuator_config/set_command_mode", &ActuatorConfigSimulationServices::SetCommandMode, this);
	m_serviceActuatorConfig_ClearFaults = m_node_handle.advertiseService("actuator_config/clear_faults", &ActuatorConfigSimulationServices::ActuatorConfig_ClearFaults, this);
	m_serviceSetServoing = m_node_handle.advertiseService("actuator_config/set_servoing", &ActuatorConfigSimulationServices::SetServoing, this);
	m_serviceMoveToPosition = m_node_handle.advertiseService("actuator_config/move_to_position", &ActuatorConfigSimulationServices::MoveToPosition, this);
	m_serviceGetCommandMode = m_node_handle.advertiseService("actuator_config/get_command_mode", &ActuatorConfigSimulationServices::GetCommandMode, this);
	m_serviceGetServoing = m_node_handle.advertiseService("actuator_config/get_servoing", &ActuatorConfigSimulationServices::GetServoing, this);
	m_serviceGetTorqueOffset = m_node_handle.advertiseService("actuator_config/get_torque_offset", &ActuatorConfigSimulationServices::GetTorqueOffset, this);
	m_serviceSetCoggingFeedforwardMode = m_node_handle.advertiseService("actuator_config/set_cogging_feedforward_mode", &ActuatorConfigSimulationServices::SetCoggingFeedforwardMode, this);
	m_serviceGetCoggingFeedforwardMode = m_node_handle.advertiseService("actuator_config/get_cogging_feedforward_mode", &ActuatorConfigSimulationServices::GetCoggingFeedforwardMode, this);
}

bool ActuatorConfigSimulationServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool ActuatorConfigSimulationServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool ActuatorConfigSimulationServices::GetAxisOffsets(kortex_driver::GetAxisOffsets::Request  &req, kortex_driver::GetAxisOffsets::Response &res)
{
	
	
	if (GetAxisOffsetsHandler)
	{
		res = GetAxisOffsetsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_axis_offsets is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetAxisOffsets(kortex_driver::SetAxisOffsets::Request  &req, kortex_driver::SetAxisOffsets::Response &res)
{
	
	
	if (SetAxisOffsetsHandler)
	{
		res = SetAxisOffsetsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_axis_offsets is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetTorqueOffset(kortex_driver::SetTorqueOffset::Request  &req, kortex_driver::SetTorqueOffset::Response &res)
{
	
	
	if (SetTorqueOffsetHandler)
	{
		res = SetTorqueOffsetHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_torque_offset is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::ActuatorConfig_GetControlMode(kortex_driver::ActuatorConfig_GetControlMode::Request  &req, kortex_driver::ActuatorConfig_GetControlMode::Response &res)
{
	
	
	if (ActuatorConfig_GetControlModeHandler)
	{
		res = ActuatorConfig_GetControlModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetControlMode(kortex_driver::SetControlMode::Request  &req, kortex_driver::SetControlMode::Response &res)
{
	
	
	if (SetControlModeHandler)
	{
		res = SetControlModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetActivatedControlLoop(kortex_driver::GetActivatedControlLoop::Request  &req, kortex_driver::GetActivatedControlLoop::Response &res)
{
	
	
	if (GetActivatedControlLoopHandler)
	{
		res = GetActivatedControlLoopHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_activated_control_loop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetActivatedControlLoop(kortex_driver::SetActivatedControlLoop::Request  &req, kortex_driver::SetActivatedControlLoop::Response &res)
{
	
	
	if (SetActivatedControlLoopHandler)
	{
		res = SetActivatedControlLoopHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_activated_control_loop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetControlLoopParameters(kortex_driver::GetControlLoopParameters::Request  &req, kortex_driver::GetControlLoopParameters::Response &res)
{
	
	
	if (GetControlLoopParametersHandler)
	{
		res = GetControlLoopParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_control_loop_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetControlLoopParameters(kortex_driver::SetControlLoopParameters::Request  &req, kortex_driver::SetControlLoopParameters::Response &res)
{
	
	
	if (SetControlLoopParametersHandler)
	{
		res = SetControlLoopParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_control_loop_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SelectCustomData(kortex_driver::SelectCustomData::Request  &req, kortex_driver::SelectCustomData::Response &res)
{
	
	
	if (SelectCustomDataHandler)
	{
		res = SelectCustomDataHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/select_custom_data is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetSelectedCustomData(kortex_driver::GetSelectedCustomData::Request  &req, kortex_driver::GetSelectedCustomData::Response &res)
{
	
	
	if (GetSelectedCustomDataHandler)
	{
		res = GetSelectedCustomDataHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_selected_custom_data is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetCommandMode(kortex_driver::SetCommandMode::Request  &req, kortex_driver::SetCommandMode::Response &res)
{
	
	
	if (SetCommandModeHandler)
	{
		res = SetCommandModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_command_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::ActuatorConfig_ClearFaults(kortex_driver::ActuatorConfig_ClearFaults::Request  &req, kortex_driver::ActuatorConfig_ClearFaults::Response &res)
{
	
	
	if (ActuatorConfig_ClearFaultsHandler)
	{
		res = ActuatorConfig_ClearFaultsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/clear_faults is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetServoing(kortex_driver::SetServoing::Request  &req, kortex_driver::SetServoing::Response &res)
{
	
	
	if (SetServoingHandler)
	{
		res = SetServoingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_servoing is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::MoveToPosition(kortex_driver::MoveToPosition::Request  &req, kortex_driver::MoveToPosition::Response &res)
{
	
	
	if (MoveToPositionHandler)
	{
		res = MoveToPositionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/move_to_position is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetCommandMode(kortex_driver::GetCommandMode::Request  &req, kortex_driver::GetCommandMode::Response &res)
{
	
	
	if (GetCommandModeHandler)
	{
		res = GetCommandModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_command_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetServoing(kortex_driver::GetServoing::Request  &req, kortex_driver::GetServoing::Response &res)
{
	
	
	if (GetServoingHandler)
	{
		res = GetServoingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_servoing is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetTorqueOffset(kortex_driver::GetTorqueOffset::Request  &req, kortex_driver::GetTorqueOffset::Response &res)
{
	
	
	if (GetTorqueOffsetHandler)
	{
		res = GetTorqueOffsetHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_torque_offset is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::SetCoggingFeedforwardMode(kortex_driver::SetCoggingFeedforwardMode::Request  &req, kortex_driver::SetCoggingFeedforwardMode::Response &res)
{
	
	
	if (SetCoggingFeedforwardModeHandler)
	{
		res = SetCoggingFeedforwardModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/set_cogging_feedforward_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool ActuatorConfigSimulationServices::GetCoggingFeedforwardMode(kortex_driver::GetCoggingFeedforwardMode::Request  &req, kortex_driver::GetCoggingFeedforwardMode::Response &res)
{
	
	
	if (GetCoggingFeedforwardModeHandler)
	{
		res = GetCoggingFeedforwardModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for actuator_config/get_cogging_feedforward_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}
