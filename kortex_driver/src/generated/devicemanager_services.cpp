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
#include "kortex_driver/generated/devicemanager_services.h"

DeviceManagerServices::DeviceManagerServices(ros::NodeHandle& n, Kinova::Api::DeviceManager::DeviceManagerClient* devicemanager, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_devicemanager(devicemanager),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);

	m_serviceSetDeviceID = n.advertiseService("device_manager/set_device_id", &DeviceManagerServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("device_manager/set_api_options", &DeviceManagerServices::SetApiOptions, this);

	m_serviceReadAllDevices = m_n.advertiseService("device_manager/read_all_devices", &DeviceManagerServices::ReadAllDevices, this);
}

bool DeviceManagerServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool DeviceManagerServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool DeviceManagerServices::ReadAllDevices(kortex_driver::ReadAllDevices::Request  &req, kortex_driver::ReadAllDevices::Response &res)
{
	Kinova::Api::DeviceManager::DeviceHandles output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_devicemanager->ReadAllDevices(m_current_device_id, m_api_options);
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
