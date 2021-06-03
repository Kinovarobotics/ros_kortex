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
 
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"

int ToRosData(Kinova::Api::InterconnectConfig::EthernetDeviceIdentification input, kortex_driver::EthernetDeviceIdentification &output)
{
	
	output.device = input.device();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::EthernetConfiguration input, kortex_driver::EthernetConfiguration &output)
{
	
	output.device = input.device();
	output.enabled = input.enabled();
	output.speed = input.speed();
	output.duplex = input.duplex();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::GPIOIdentification input, kortex_driver::GPIOIdentification &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::GPIOConfiguration input, kortex_driver::InterconnectConfig_GPIOConfiguration &output)
{
	
	output.identifier = input.identifier();
	output.mode = input.mode();
	output.pull = input.pull();
	output.default_value = input.default_value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::GPIOState input, kortex_driver::GPIOState &output)
{
	
	output.identifier = input.identifier();
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CDeviceIdentification input, kortex_driver::I2CDeviceIdentification &output)
{
	
	output.device = input.device();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CConfiguration input, kortex_driver::I2CConfiguration &output)
{
	
	output.device = input.device();
	output.enabled = input.enabled();
	output.mode = input.mode();
	output.addressing = input.addressing();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CReadParameter input, kortex_driver::I2CReadParameter &output)
{
	
	output.device = input.device();
	output.device_address = input.device_address();
	output.size = input.size();
	output.timeout = input.timeout();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CReadRegisterParameter input, kortex_driver::I2CReadRegisterParameter &output)
{
	
	output.device = input.device();
	output.device_address = input.device_address();
	output.register_address = input.register_address();
	output.register_address_size = input.register_address_size();
	output.size = input.size();
	output.timeout = input.timeout();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CWriteParameter input, kortex_driver::I2CWriteParameter &output)
{
	
	output.device = input.device();
	output.device_address = input.device_address();
	output.timeout = input.timeout();
	ToRosData(input.data(), output.data);

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter input, kortex_driver::I2CWriteRegisterParameter &output)
{
	
	output.device = input.device();
	output.device_address = input.device_address();
	output.register_address = input.register_address();
	output.register_address_size = input.register_address_size();
	output.timeout = input.timeout();
	ToRosData(input.data(), output.data);

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectConfig::I2CData input, kortex_driver::I2CData &output)
{
	
	output.data = std::vector<uint8_t>(input.data().begin(), input.data().end());
	output.size = input.size();

	
	
	return 0;
}
