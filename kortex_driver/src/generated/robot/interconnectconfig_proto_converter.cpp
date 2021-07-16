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
 
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"

int ToProtoData(kortex_driver::EthernetDeviceIdentification input, Kinova::Api::InterconnectConfig::EthernetDeviceIdentification *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::EthernetDevice)input.device);
	
	return 0;
}
int ToProtoData(kortex_driver::EthernetConfiguration input, Kinova::Api::InterconnectConfig::EthernetConfiguration *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::EthernetDevice)input.device);
	output->set_enabled(input.enabled);
	output->set_speed((Kinova::Api::InterconnectConfig::EthernetSpeed)input.speed);
	output->set_duplex((Kinova::Api::InterconnectConfig::EthernetDuplex)input.duplex);
	
	return 0;
}
int ToProtoData(kortex_driver::GPIOIdentification input, Kinova::Api::InterconnectConfig::GPIOIdentification *output)
{
	
	output->set_identifier((Kinova::Api::InterconnectConfig::GPIOIdentifier)input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::InterconnectConfig_GPIOConfiguration input, Kinova::Api::InterconnectConfig::GPIOConfiguration *output)
{
	
	output->set_identifier((Kinova::Api::InterconnectConfig::GPIOIdentifier)input.identifier);
	output->set_mode((Kinova::Api::InterconnectConfig::GPIOMode)input.mode);
	output->set_pull((Kinova::Api::InterconnectConfig::GPIOPull)input.pull);
	output->set_default_value((Kinova::Api::InterconnectConfig::GPIOValue)input.default_value);
	
	return 0;
}
int ToProtoData(kortex_driver::GPIOState input, Kinova::Api::InterconnectConfig::GPIOState *output)
{
	
	output->set_identifier((Kinova::Api::InterconnectConfig::GPIOIdentifier)input.identifier);
	output->set_value((Kinova::Api::InterconnectConfig::GPIOValue)input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::I2CDeviceIdentification input, Kinova::Api::InterconnectConfig::I2CDeviceIdentification *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::I2CDevice)input.device);
	
	return 0;
}
int ToProtoData(kortex_driver::I2CConfiguration input, Kinova::Api::InterconnectConfig::I2CConfiguration *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::I2CDevice)input.device);
	output->set_enabled(input.enabled);
	output->set_mode((Kinova::Api::InterconnectConfig::I2CMode)input.mode);
	output->set_addressing((Kinova::Api::InterconnectConfig::I2CDeviceAddressing)input.addressing);
	
	return 0;
}
int ToProtoData(kortex_driver::I2CReadParameter input, Kinova::Api::InterconnectConfig::I2CReadParameter *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::I2CDevice)input.device);
	output->set_device_address(input.device_address);
	output->set_size(input.size);
	output->set_timeout(input.timeout);
	
	return 0;
}
int ToProtoData(kortex_driver::I2CReadRegisterParameter input, Kinova::Api::InterconnectConfig::I2CReadRegisterParameter *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::I2CDevice)input.device);
	output->set_device_address(input.device_address);
	output->set_register_address(input.register_address);
	output->set_register_address_size((Kinova::Api::InterconnectConfig::I2CRegisterAddressSize)input.register_address_size);
	output->set_size(input.size);
	output->set_timeout(input.timeout);
	
	return 0;
}
int ToProtoData(kortex_driver::I2CWriteParameter input, Kinova::Api::InterconnectConfig::I2CWriteParameter *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::I2CDevice)input.device);
	output->set_device_address(input.device_address);
	output->set_timeout(input.timeout); 
	ToProtoData(input.data, output->mutable_data());
	
	return 0;
}
int ToProtoData(kortex_driver::I2CWriteRegisterParameter input, Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter *output)
{
	
	output->set_device((Kinova::Api::InterconnectConfig::I2CDevice)input.device);
	output->set_device_address(input.device_address);
	output->set_register_address(input.register_address);
	output->set_register_address_size((Kinova::Api::InterconnectConfig::I2CRegisterAddressSize)input.register_address_size);
	output->set_timeout(input.timeout); 
	ToProtoData(input.data, output->mutable_data());
	
	return 0;
}
int ToProtoData(kortex_driver::I2CData input, Kinova::Api::InterconnectConfig::I2CData *output)
{
	
	output->set_data(std::string(input.data.begin(), input.data.end()));
	output->set_size(input.size);
	
	return 0;
}
