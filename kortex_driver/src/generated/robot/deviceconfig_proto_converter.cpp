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
 
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"

int ToProtoData(kortex_driver::DeviceType input, Kinova::Api::DeviceConfig::DeviceType *output)
{
	
	output->set_device_type((Kinova::Api::Common::DeviceTypes)input.device_type);
	
	return 0;
}
int ToProtoData(kortex_driver::RunMode input, Kinova::Api::DeviceConfig::RunMode *output)
{
	
	output->set_run_mode((Kinova::Api::DeviceConfig::RunModes)input.run_mode);
	
	return 0;
}
int ToProtoData(kortex_driver::FirmwareVersion input, Kinova::Api::DeviceConfig::FirmwareVersion *output)
{
	
	output->set_firmware_version(input.firmware_version);
	
	return 0;
}
int ToProtoData(kortex_driver::BootloaderVersion input, Kinova::Api::DeviceConfig::BootloaderVersion *output)
{
	
	output->set_bootloader_version(input.bootloader_version);
	
	return 0;
}
int ToProtoData(kortex_driver::ModelNumber input, Kinova::Api::DeviceConfig::ModelNumber *output)
{
	
	output->set_model_number(input.model_number);
	
	return 0;
}
int ToProtoData(kortex_driver::PartNumber input, Kinova::Api::DeviceConfig::PartNumber *output)
{
	
	output->set_part_number(input.part_number);
	
	return 0;
}
int ToProtoData(kortex_driver::SerialNumber input, Kinova::Api::DeviceConfig::SerialNumber *output)
{
	
	output->set_serial_number(input.serial_number);
	
	return 0;
}
int ToProtoData(kortex_driver::MACAddress input, Kinova::Api::DeviceConfig::MACAddress *output)
{
	
	output->set_mac_address(std::string(input.mac_address.begin(), input.mac_address.end()));
	
	return 0;
}
int ToProtoData(kortex_driver::IPv4Settings input, Kinova::Api::DeviceConfig::IPv4Settings *output)
{
	
	output->set_ipv4_address(input.ipv4_address);
	output->set_ipv4_subnet_mask(input.ipv4_subnet_mask);
	output->set_ipv4_default_gateway(input.ipv4_default_gateway);
	
	return 0;
}
int ToProtoData(kortex_driver::PartNumberRevision input, Kinova::Api::DeviceConfig::PartNumberRevision *output)
{
	
	output->set_part_number_revision(input.part_number_revision);
	
	return 0;
}
int ToProtoData(kortex_driver::PowerOnSelfTestResult input, Kinova::Api::DeviceConfig::PowerOnSelfTestResult *output)
{
	
	output->set_power_on_self_test_result(input.power_on_self_test_result);
	
	return 0;
}
int ToProtoData(kortex_driver::RebootRqst input, Kinova::Api::DeviceConfig::RebootRqst *output)
{
	
	output->set_delay(input.delay);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyInformation input, Kinova::Api::DeviceConfig::SafetyInformation *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_can_change_safety_state(input.can_change_safety_state);
	output->set_has_warning_threshold(input.has_warning_threshold);
	output->set_has_error_threshold(input.has_error_threshold);
	output->set_limit_type((Kinova::Api::DeviceConfig::SafetyLimitType)input.limit_type);
	output->set_default_warning_threshold(input.default_warning_threshold);
	output->set_default_error_threshold(input.default_error_threshold);
	output->set_upper_hard_limit(input.upper_hard_limit);
	output->set_lower_hard_limit(input.lower_hard_limit);
	output->set_status((Kinova::Api::Common::SafetyStatusValue)input.status);
	output->set_unit((Kinova::Api::Common::Unit)input.unit);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyInformationList input, Kinova::Api::DeviceConfig::SafetyInformationList *output)
{
	 
	output->clear_information();
	for(int i = 0; i < input.information.size(); i++)
	{
		ToProtoData(input.information[i], output->add_information());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyEnable input, Kinova::Api::DeviceConfig::SafetyEnable *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_enable(input.enable);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyThreshold input, Kinova::Api::DeviceConfig::SafetyThreshold *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyConfiguration input, Kinova::Api::DeviceConfig::SafetyConfiguration *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_error_threshold(input.error_threshold);
	output->set_warning_threshold(input.warning_threshold); 
	ToProtoData(input.enable, output->mutable_enable());
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyConfigurationList input, Kinova::Api::DeviceConfig::SafetyConfigurationList *output)
{
	 
	output->clear_configuration();
	for(int i = 0; i < input.configuration.size(); i++)
	{
		ToProtoData(input.configuration[i], output->add_configuration());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyStatus input, Kinova::Api::DeviceConfig::SafetyStatus *output)
{
	
	output->set_value((Kinova::Api::Common::SafetyStatusValue)input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::CalibrationParameter input, Kinova::Api::DeviceConfig::CalibrationParameter *output)
{
	
	output->set_calibration_parameter_identifier(input.calibration_parameter_identifier);
	if(input.oneof_value.signedIntValue.size() > 0)
	{
		output->set_signedintvalue(input.oneof_value.signedIntValue[0]);
	}
	if(input.oneof_value.unsignedIntValue.size() > 0)
	{
		output->set_unsignedintvalue(input.oneof_value.unsignedIntValue[0]);
	}
	if(input.oneof_value.floatValue.size() > 0)
	{
		output->set_floatvalue(input.oneof_value.floatValue[0]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Calibration input, Kinova::Api::DeviceConfig::Calibration *output)
{
	
	output->set_calibration_item((Kinova::Api::DeviceConfig::CalibrationItem)input.calibration_item); 
	output->clear_calibration_parameter();
	for(int i = 0; i < input.calibration_parameter.size(); i++)
	{
		ToProtoData(input.calibration_parameter[i], output->add_calibration_parameter());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::CalibrationElement input, Kinova::Api::DeviceConfig::CalibrationElement *output)
{
	
	output->set_calibration_item((Kinova::Api::DeviceConfig::CalibrationItem)input.calibration_item);
	
	return 0;
}
int ToProtoData(kortex_driver::CalibrationResult input, Kinova::Api::DeviceConfig::CalibrationResult *output)
{
	
	output->set_calibration_status((Kinova::Api::DeviceConfig::CalibrationStatus)input.calibration_status);
	output->set_calibration_details(input.calibration_details);
	
	return 0;
}
int ToProtoData(kortex_driver::DeviceConfig_CapSenseConfig input, Kinova::Api::DeviceConfig::CapSenseConfig *output)
{
	
	output->set_mode((Kinova::Api::DeviceConfig::CapSenseMode)input.mode);
	output->set_threshold_a(input.threshold_a);
	output->set_threshold_b(input.threshold_b);
	
	return 0;
}
int ToProtoData(kortex_driver::CapSenseRegister input, Kinova::Api::DeviceConfig::CapSenseRegister *output)
{
	
	output->set_address(input.address);
	output->set_value(input.value);
	
	return 0;
}
