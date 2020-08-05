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
#include "kortex_driver/generated/robot/interconnectconfig_services.h"

InterconnectConfigRobotServices::InterconnectConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::InterconnectConfig::InterconnectConfigClient* interconnectconfig, uint32_t device_id, uint32_t timeout_ms): 
	IInterconnectConfigServices(node_handle),
	m_interconnectconfig(interconnectconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = m_node_handle.advertiseService("interconnect_config/set_device_id", &InterconnectConfigRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("interconnect_config/set_api_options", &InterconnectConfigRobotServices::SetApiOptions, this);

	m_serviceGetUARTConfiguration = m_node_handle.advertiseService("interconnect_config/get_u_a_r_t_configuration", &InterconnectConfigRobotServices::GetUARTConfiguration, this);
	m_serviceSetUARTConfiguration = m_node_handle.advertiseService("interconnect_config/set_u_a_r_t_configuration", &InterconnectConfigRobotServices::SetUARTConfiguration, this);
	m_serviceGetEthernetConfiguration = m_node_handle.advertiseService("interconnect_config/get_ethernet_configuration", &InterconnectConfigRobotServices::GetEthernetConfiguration, this);
	m_serviceSetEthernetConfiguration = m_node_handle.advertiseService("interconnect_config/set_ethernet_configuration", &InterconnectConfigRobotServices::SetEthernetConfiguration, this);
	m_serviceGetGPIOConfiguration = m_node_handle.advertiseService("interconnect_config/get_g_p_i_o_configuration", &InterconnectConfigRobotServices::GetGPIOConfiguration, this);
	m_serviceSetGPIOConfiguration = m_node_handle.advertiseService("interconnect_config/set_g_p_i_o_configuration", &InterconnectConfigRobotServices::SetGPIOConfiguration, this);
	m_serviceGetGPIOState = m_node_handle.advertiseService("interconnect_config/get_g_p_i_o_state", &InterconnectConfigRobotServices::GetGPIOState, this);
	m_serviceSetGPIOState = m_node_handle.advertiseService("interconnect_config/set_g_p_i_o_state", &InterconnectConfigRobotServices::SetGPIOState, this);
	m_serviceGetI2CConfiguration = m_node_handle.advertiseService("interconnect_config/get_i2_c_configuration", &InterconnectConfigRobotServices::GetI2CConfiguration, this);
	m_serviceSetI2CConfiguration = m_node_handle.advertiseService("interconnect_config/set_i2_c_configuration", &InterconnectConfigRobotServices::SetI2CConfiguration, this);
	m_serviceI2CRead = m_node_handle.advertiseService("interconnect_config/i2_c_read", &InterconnectConfigRobotServices::I2CRead, this);
	m_serviceI2CReadRegister = m_node_handle.advertiseService("interconnect_config/i2_c_read_register", &InterconnectConfigRobotServices::I2CReadRegister, this);
	m_serviceI2CWrite = m_node_handle.advertiseService("interconnect_config/i2_c_write", &InterconnectConfigRobotServices::I2CWrite, this);
	m_serviceI2CWriteRegister = m_node_handle.advertiseService("interconnect_config/i2_c_write_register", &InterconnectConfigRobotServices::I2CWriteRegister, this);
}

bool InterconnectConfigRobotServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool InterconnectConfigRobotServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool InterconnectConfigRobotServices::GetUARTConfiguration(kortex_driver::GetUARTConfiguration::Request  &req, kortex_driver::GetUARTConfiguration::Response &res)
{
	
	Kinova::Api::Common::UARTDeviceIdentification input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::UARTConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetUARTConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetUARTConfiguration(kortex_driver::SetUARTConfiguration::Request  &req, kortex_driver::SetUARTConfiguration::Response &res)
{
	
	Kinova::Api::Common::UARTConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetUARTConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetEthernetConfiguration(kortex_driver::GetEthernetConfiguration::Request  &req, kortex_driver::GetEthernetConfiguration::Response &res)
{
	
	Kinova::Api::InterconnectConfig::EthernetDeviceIdentification input;
	ToProtoData(req.input, &input);
	Kinova::Api::InterconnectConfig::EthernetConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetEthernetConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetEthernetConfiguration(kortex_driver::SetEthernetConfiguration::Request  &req, kortex_driver::SetEthernetConfiguration::Response &res)
{
	
	Kinova::Api::InterconnectConfig::EthernetConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetEthernetConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetGPIOConfiguration(kortex_driver::GetGPIOConfiguration::Request  &req, kortex_driver::GetGPIOConfiguration::Response &res)
{
	
	Kinova::Api::InterconnectConfig::GPIOIdentification input;
	ToProtoData(req.input, &input);
	Kinova::Api::InterconnectConfig::GPIOConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetGPIOConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetGPIOConfiguration(kortex_driver::SetGPIOConfiguration::Request  &req, kortex_driver::SetGPIOConfiguration::Response &res)
{
	
	Kinova::Api::InterconnectConfig::GPIOConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetGPIOConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetGPIOState(kortex_driver::GetGPIOState::Request  &req, kortex_driver::GetGPIOState::Response &res)
{
	
	Kinova::Api::InterconnectConfig::GPIOIdentification input;
	ToProtoData(req.input, &input);
	Kinova::Api::InterconnectConfig::GPIOState output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetGPIOState(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetGPIOState(kortex_driver::SetGPIOState::Request  &req, kortex_driver::SetGPIOState::Response &res)
{
	
	Kinova::Api::InterconnectConfig::GPIOState input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetGPIOState(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::GetI2CConfiguration(kortex_driver::GetI2CConfiguration::Request  &req, kortex_driver::GetI2CConfiguration::Response &res)
{
	
	Kinova::Api::InterconnectConfig::I2CDeviceIdentification input;
	ToProtoData(req.input, &input);
	Kinova::Api::InterconnectConfig::I2CConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->GetI2CConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::SetI2CConfiguration(kortex_driver::SetI2CConfiguration::Request  &req, kortex_driver::SetI2CConfiguration::Response &res)
{
	
	Kinova::Api::InterconnectConfig::I2CConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->SetI2CConfiguration(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CRead(kortex_driver::I2CRead::Request  &req, kortex_driver::I2CRead::Response &res)
{
	
	Kinova::Api::InterconnectConfig::I2CReadParameter input;
	ToProtoData(req.input, &input);
	Kinova::Api::InterconnectConfig::I2CData output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->I2CRead(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CReadRegister(kortex_driver::I2CReadRegister::Request  &req, kortex_driver::I2CReadRegister::Response &res)
{
	
	Kinova::Api::InterconnectConfig::I2CReadRegisterParameter input;
	ToProtoData(req.input, &input);
	Kinova::Api::InterconnectConfig::I2CData output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_interconnectconfig->I2CReadRegister(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CWrite(kortex_driver::I2CWrite::Request  &req, kortex_driver::I2CWrite::Response &res)
{
	
	Kinova::Api::InterconnectConfig::I2CWriteParameter input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->I2CWrite(input, m_current_device_id, m_api_options);
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

bool InterconnectConfigRobotServices::I2CWriteRegister(kortex_driver::I2CWriteRegister::Request  &req, kortex_driver::I2CWriteRegister::Response &res)
{
	
	Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_interconnectconfig->I2CWriteRegister(input, m_current_device_id, m_api_options);
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
