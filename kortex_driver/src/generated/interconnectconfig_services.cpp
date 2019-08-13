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
#include "kortex_driver/generated/interconnectconfig_services.h"

InterconnectConfigServices::InterconnectConfigServices(ros::NodeHandle& n, Kinova::Api::InterconnectConfig::InterconnectConfigClient* interconnectconfig, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_interconnectconfig(interconnectconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = n.advertiseService("interconnect_config/set_device_id", &InterconnectConfigServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("interconnect_config/set_api_options", &InterconnectConfigServices::SetApiOptions, this);

	m_serviceGetUARTConfiguration = m_n.advertiseService("interconnect_config/get_u_a_r_t_configuration", &InterconnectConfigServices::GetUARTConfiguration, this);
	m_serviceSetUARTConfiguration = m_n.advertiseService("interconnect_config/set_u_a_r_t_configuration", &InterconnectConfigServices::SetUARTConfiguration, this);
	m_serviceGetEthernetConfiguration = m_n.advertiseService("interconnect_config/get_ethernet_configuration", &InterconnectConfigServices::GetEthernetConfiguration, this);
	m_serviceSetEthernetConfiguration = m_n.advertiseService("interconnect_config/set_ethernet_configuration", &InterconnectConfigServices::SetEthernetConfiguration, this);
	m_serviceGetGPIOConfiguration = m_n.advertiseService("interconnect_config/get_g_p_i_o_configuration", &InterconnectConfigServices::GetGPIOConfiguration, this);
	m_serviceSetGPIOConfiguration = m_n.advertiseService("interconnect_config/set_g_p_i_o_configuration", &InterconnectConfigServices::SetGPIOConfiguration, this);
	m_serviceGetGPIOState = m_n.advertiseService("interconnect_config/get_g_p_i_o_state", &InterconnectConfigServices::GetGPIOState, this);
	m_serviceSetGPIOState = m_n.advertiseService("interconnect_config/set_g_p_i_o_state", &InterconnectConfigServices::SetGPIOState, this);
	m_serviceGetI2CConfiguration = m_n.advertiseService("interconnect_config/get_i2_c_configuration", &InterconnectConfigServices::GetI2CConfiguration, this);
	m_serviceSetI2CConfiguration = m_n.advertiseService("interconnect_config/set_i2_c_configuration", &InterconnectConfigServices::SetI2CConfiguration, this);
	m_serviceI2CRead = m_n.advertiseService("interconnect_config/i2_c_read", &InterconnectConfigServices::I2CRead, this);
	m_serviceI2CReadRegister = m_n.advertiseService("interconnect_config/i2_c_read_register", &InterconnectConfigServices::I2CReadRegister, this);
	m_serviceI2CWrite = m_n.advertiseService("interconnect_config/i2_c_write", &InterconnectConfigServices::I2CWrite, this);
	m_serviceI2CWriteRegister = m_n.advertiseService("interconnect_config/i2_c_write_register", &InterconnectConfigServices::I2CWriteRegister, this);
}

bool InterconnectConfigServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool InterconnectConfigServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool InterconnectConfigServices::GetUARTConfiguration(kortex_driver::GetUARTConfiguration::Request  &req, kortex_driver::GetUARTConfiguration::Response &res)
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

bool InterconnectConfigServices::SetUARTConfiguration(kortex_driver::SetUARTConfiguration::Request  &req, kortex_driver::SetUARTConfiguration::Response &res)
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

bool InterconnectConfigServices::GetEthernetConfiguration(kortex_driver::GetEthernetConfiguration::Request  &req, kortex_driver::GetEthernetConfiguration::Response &res)
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

bool InterconnectConfigServices::SetEthernetConfiguration(kortex_driver::SetEthernetConfiguration::Request  &req, kortex_driver::SetEthernetConfiguration::Response &res)
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

bool InterconnectConfigServices::GetGPIOConfiguration(kortex_driver::GetGPIOConfiguration::Request  &req, kortex_driver::GetGPIOConfiguration::Response &res)
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

bool InterconnectConfigServices::SetGPIOConfiguration(kortex_driver::SetGPIOConfiguration::Request  &req, kortex_driver::SetGPIOConfiguration::Response &res)
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

bool InterconnectConfigServices::GetGPIOState(kortex_driver::GetGPIOState::Request  &req, kortex_driver::GetGPIOState::Response &res)
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

bool InterconnectConfigServices::SetGPIOState(kortex_driver::SetGPIOState::Request  &req, kortex_driver::SetGPIOState::Response &res)
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

bool InterconnectConfigServices::GetI2CConfiguration(kortex_driver::GetI2CConfiguration::Request  &req, kortex_driver::GetI2CConfiguration::Response &res)
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

bool InterconnectConfigServices::SetI2CConfiguration(kortex_driver::SetI2CConfiguration::Request  &req, kortex_driver::SetI2CConfiguration::Response &res)
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

bool InterconnectConfigServices::I2CRead(kortex_driver::I2CRead::Request  &req, kortex_driver::I2CRead::Response &res)
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

bool InterconnectConfigServices::I2CReadRegister(kortex_driver::I2CReadRegister::Request  &req, kortex_driver::I2CReadRegister::Response &res)
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

bool InterconnectConfigServices::I2CWrite(kortex_driver::I2CWrite::Request  &req, kortex_driver::I2CWrite::Response &res)
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

bool InterconnectConfigServices::I2CWriteRegister(kortex_driver::I2CWriteRegister::Request  &req, kortex_driver::I2CWriteRegister::Response &res)
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
