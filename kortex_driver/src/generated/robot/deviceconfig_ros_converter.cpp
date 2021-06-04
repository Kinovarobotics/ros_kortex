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
 
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"

int ToRosData(Kinova::Api::DeviceConfig::DeviceType input, kortex_driver::DeviceType &output)
{
	
	output.device_type = input.device_type();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::RunMode input, kortex_driver::RunMode &output)
{
	
	output.run_mode = input.run_mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::FirmwareVersion input, kortex_driver::FirmwareVersion &output)
{
	
	output.firmware_version = input.firmware_version();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::BootloaderVersion input, kortex_driver::BootloaderVersion &output)
{
	
	output.bootloader_version = input.bootloader_version();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::ModelNumber input, kortex_driver::ModelNumber &output)
{
	
	output.model_number = input.model_number();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::PartNumber input, kortex_driver::PartNumber &output)
{
	
	output.part_number = input.part_number();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SerialNumber input, kortex_driver::SerialNumber &output)
{
	
	output.serial_number = input.serial_number();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::MACAddress input, kortex_driver::MACAddress &output)
{
	
	output.mac_address = std::vector<uint8_t>(input.mac_address().begin(), input.mac_address().end());

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::IPv4Settings input, kortex_driver::IPv4Settings &output)
{
	
	output.ipv4_address = input.ipv4_address();
	output.ipv4_subnet_mask = input.ipv4_subnet_mask();
	output.ipv4_default_gateway = input.ipv4_default_gateway();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::PartNumberRevision input, kortex_driver::PartNumberRevision &output)
{
	
	output.part_number_revision = input.part_number_revision();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::PowerOnSelfTestResult input, kortex_driver::PowerOnSelfTestResult &output)
{
	
	output.power_on_self_test_result = input.power_on_self_test_result();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::RebootRqst input, kortex_driver::RebootRqst &output)
{
	
	output.delay = input.delay();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SafetyInformation input, kortex_driver::SafetyInformation &output)
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
int ToRosData(Kinova::Api::DeviceConfig::SafetyInformationList input, kortex_driver::SafetyInformationList &output)
{
	
	output.information.clear();
	for(int i = 0; i < input.information_size(); i++)
	{
		decltype(output.information)::value_type temp;
		ToRosData(input.information(i), temp);
		output.information.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SafetyEnable input, kortex_driver::SafetyEnable &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.enable = input.enable();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SafetyThreshold input, kortex_driver::SafetyThreshold &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SafetyConfiguration input, kortex_driver::SafetyConfiguration &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.error_threshold = input.error_threshold();
	output.warning_threshold = input.warning_threshold();
	ToRosData(input.enable(), output.enable);

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SafetyConfigurationList input, kortex_driver::SafetyConfigurationList &output)
{
	
	output.configuration.clear();
	for(int i = 0; i < input.configuration_size(); i++)
	{
		decltype(output.configuration)::value_type temp;
		ToRosData(input.configuration(i), temp);
		output.configuration.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::SafetyStatus input, kortex_driver::SafetyStatus &output)
{
	
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::CalibrationParameter input, kortex_driver::CalibrationParameter &output)
{
	
	output.calibration_parameter_identifier = input.calibration_parameter_identifier();

	
	auto oneof_type_value = input.value_case();
	switch(oneof_type_value)
	{ 
	
		case Kinova::Api::DeviceConfig::CalibrationParameter::kSignedIntValue:
		{
			break;
		} 
	
		case Kinova::Api::DeviceConfig::CalibrationParameter::kUnsignedIntValue:
		{
			break;
		} 
	
		case Kinova::Api::DeviceConfig::CalibrationParameter::kFloatValue:
		{
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::Calibration input, kortex_driver::Calibration &output)
{
	
	output.calibration_item = input.calibration_item();
	output.calibration_parameter.clear();
	for(int i = 0; i < input.calibration_parameter_size(); i++)
	{
		decltype(output.calibration_parameter)::value_type temp;
		ToRosData(input.calibration_parameter(i), temp);
		output.calibration_parameter.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::CalibrationElement input, kortex_driver::CalibrationElement &output)
{
	
	output.calibration_item = input.calibration_item();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::CalibrationResult input, kortex_driver::CalibrationResult &output)
{
	
	output.calibration_status = input.calibration_status();
	output.calibration_details = input.calibration_details();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::CapSenseConfig input, kortex_driver::DeviceConfig_CapSenseConfig &output)
{
	
	output.mode = input.mode();
	output.threshold_a = input.threshold_a();
	output.threshold_b = input.threshold_b();

	
	
	return 0;
}
int ToRosData(Kinova::Api::DeviceConfig::CapSenseRegister input, kortex_driver::CapSenseRegister &output)
{
	
	output.address = input.address();
	output.value = input.value();

	
	
	return 0;
}
