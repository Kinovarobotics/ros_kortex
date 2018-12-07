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
 
#include "deviceconfig_proto_converter.h"

#include "common_proto_converter.h"


int ToProtoData(kortex_device_manager::DeviceType input, DeviceType *output)
{
	output->set_device_type((Kinova::Api::Common::DeviceTypes)input.device_type);

	return 0;
}
int ToProtoData(kortex_device_manager::RunMode input, RunMode *output)
{
	output->set_run_mode((Kinova::Api::DeviceConfig::RunModes)input.run_mode);

	return 0;
}
int ToProtoData(kortex_device_manager::FirmwareVersion input, FirmwareVersion *output)
{
	output->set_firmware_version(input.firmware_version);

	return 0;
}
int ToProtoData(kortex_device_manager::BootloaderVersion input, BootloaderVersion *output)
{
	output->set_bootloader_version(input.bootloader_version);

	return 0;
}
int ToProtoData(kortex_device_manager::ModelNumber input, ModelNumber *output)
{
	output->set_model_number(input.model_number);

	return 0;
}
int ToProtoData(kortex_device_manager::PartNumber input, PartNumber *output)
{
	output->set_part_number(input.part_number);

	return 0;
}
int ToProtoData(kortex_device_manager::SerialNumber input, SerialNumber *output)
{
	output->set_serial_number(input.serial_number);

	return 0;
}
int ToProtoData(kortex_device_manager::MACAddress input, MACAddress *output)
{
	output->set_mac_address(std::string(input.mac_address.begin(), input.mac_address.end()));

	return 0;
}
int ToProtoData(kortex_device_manager::IPv4Settings input, IPv4Settings *output)
{
	output->set_ipv4_address(input.ipv4_address);
	output->set_ipv4_subnet_mask(input.ipv4_subnet_mask);
	output->set_ipv4_default_gateway(input.ipv4_default_gateway);

	return 0;
}
int ToProtoData(kortex_device_manager::PartNumberRevision input, PartNumberRevision *output)
{
	output->set_part_number_revision(input.part_number_revision);

	return 0;
}
int ToProtoData(kortex_device_manager::PowerOnSelfTestResult input, PowerOnSelfTestResult *output)
{
	output->set_power_on_self_test_result(input.power_on_self_test_result);

	return 0;
}
int ToProtoData(kortex_device_manager::RebootRqst input, RebootRqst *output)
{
	output->set_delay(input.delay);

	return 0;
}
int ToProtoData(kortex_device_manager::SafetyInformation input, SafetyInformation *output)
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
int ToProtoData(kortex_device_manager::SafetyInformationList input, SafetyInformationList *output)
{ 
	output->clear_information();
	for(int i = 0; i < input.information.size(); i++)
	{
		ToProtoData(input.information[i], output->add_information());
	}

	return 0;
}
int ToProtoData(kortex_device_manager::SafetyEnable input, SafetyEnable *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_enable(input.enable);

	return 0;
}
int ToProtoData(kortex_device_manager::SafetyThreshold input, SafetyThreshold *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_device_manager::SafetyConfiguration input, SafetyConfiguration *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_error_threshold(input.error_threshold);
	output->set_warning_threshold(input.warning_threshold);
	ToProtoData(input.enable, output->mutable_enable());

	return 0;
}
int ToProtoData(kortex_device_manager::SafetyConfigurationList input, SafetyConfigurationList *output)
{ 
	output->clear_configuration();
	for(int i = 0; i < input.configuration.size(); i++)
	{
		ToProtoData(input.configuration[i], output->add_configuration());
	}

	return 0;
}
int ToProtoData(kortex_device_manager::SafetyStatus input, SafetyStatus *output)
{
	output->set_value((Kinova::Api::Common::SafetyStatusValue)input.value);

	return 0;
}
