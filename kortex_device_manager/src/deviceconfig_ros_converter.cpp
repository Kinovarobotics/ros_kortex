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
 
#include "deviceconfig_ros_converter.h"

#include "common_ros_converter.h"


int ToRosData(DeviceType input, kortex_device_manager::DeviceType &output)
{
	output.device_type = input.device_type();
	
	return 0;
}
int ToRosData(RunMode input, kortex_device_manager::RunMode &output)
{
	output.run_mode = input.run_mode();
	
	return 0;
}
int ToRosData(FirmwareVersion input, kortex_device_manager::FirmwareVersion &output)
{
	output.firmware_version = input.firmware_version();
	
	return 0;
}
int ToRosData(BootloaderVersion input, kortex_device_manager::BootloaderVersion &output)
{
	output.bootloader_version = input.bootloader_version();
	
	return 0;
}
int ToRosData(ModelNumber input, kortex_device_manager::ModelNumber &output)
{
	output.model_number = input.model_number();
	
	return 0;
}
int ToRosData(PartNumber input, kortex_device_manager::PartNumber &output)
{
	output.part_number = input.part_number();
	
	return 0;
}
int ToRosData(SerialNumber input, kortex_device_manager::SerialNumber &output)
{
	output.serial_number = input.serial_number();
	
	return 0;
}
int ToRosData(MACAddress input, kortex_device_manager::MACAddress &output)
{
	output.mac_address = std::vector<uint8_t>(input.mac_address().begin(), input.mac_address().end());
	
	return 0;
}
int ToRosData(IPv4Settings input, kortex_device_manager::IPv4Settings &output)
{
	output.ipv4_address = input.ipv4_address();
	output.ipv4_subnet_mask = input.ipv4_subnet_mask();
	output.ipv4_default_gateway = input.ipv4_default_gateway();
	
	return 0;
}
int ToRosData(PartNumberRevision input, kortex_device_manager::PartNumberRevision &output)
{
	output.part_number_revision = input.part_number_revision();
	
	return 0;
}
int ToRosData(PowerOnSelfTestResult input, kortex_device_manager::PowerOnSelfTestResult &output)
{
	output.power_on_self_test_result = input.power_on_self_test_result();
	
	return 0;
}
int ToRosData(RebootRqst input, kortex_device_manager::RebootRqst &output)
{
	output.delay = input.delay();
	
	return 0;
}
int ToRosData(SafetyInformation input, kortex_device_manager::SafetyInformation &output)
{
	ToRosData(input.handle(), output.handle);
	output.can_change_safety_state = input.can_change_safety_state();
	output.has_warning_threshold = input.has_warning_threshold();
	output.has_error_threshold = input.has_error_threshold();
	output.limit_type = input.limit_type();
	output.default_warning_threshold = input.default_warning_threshold();
	output.default_error_threshold = input.default_error_threshold();
	output.upper_hard_limit = input.upper_hard_limit();
	output.lower_hard_limit = input.lower_hard_limit();
	output.status = input.status();
	output.unit = input.unit();
	
	return 0;
}
int ToRosData(SafetyInformationList input, kortex_device_manager::SafetyInformationList &output)
{ 
	output.information.clear();
	for(int i = 0; i < input.information_size(); i++)
	{
		kortex_device_manager::SafetyInformation temp;
		ToRosData(input.information(i), temp);
		output.information.push_back(temp);
	}
	
	return 0;
}
int ToRosData(SafetyEnable input, kortex_device_manager::SafetyEnable &output)
{
	ToRosData(input.handle(), output.handle);
	output.enable = input.enable();
	
	return 0;
}
int ToRosData(SafetyThreshold input, kortex_device_manager::SafetyThreshold &output)
{
	ToRosData(input.handle(), output.handle);
	output.value = input.value();
	
	return 0;
}
int ToRosData(SafetyConfiguration input, kortex_device_manager::SafetyConfiguration &output)
{
	ToRosData(input.handle(), output.handle);
	output.error_threshold = input.error_threshold();
	output.warning_threshold = input.warning_threshold();
	ToRosData(input.enable(), output.enable);
	
	return 0;
}
int ToRosData(SafetyConfigurationList input, kortex_device_manager::SafetyConfigurationList &output)
{ 
	output.configuration.clear();
	for(int i = 0; i < input.configuration_size(); i++)
	{
		kortex_device_manager::SafetyConfiguration temp;
		ToRosData(input.configuration(i), temp);
		output.configuration.push_back(temp);
	}
	
	return 0;
}
int ToRosData(SafetyStatus input, kortex_device_manager::SafetyStatus &output)
{
	output.value = input.value();
	
	return 0;
}
