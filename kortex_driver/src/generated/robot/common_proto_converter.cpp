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

int ToProtoData(kortex_driver::DeviceHandle input, Kinova::Api::Common::DeviceHandle *output)
{
	
	output->set_device_type((Kinova::Api::Common::DeviceTypes)input.device_type);
	output->set_device_identifier(input.device_identifier);
	output->set_order(input.order);
	
	return 0;
}
int ToProtoData(kortex_driver::Empty input, Kinova::Api::Common::Empty *output)
{
	
	
	return 0;
}
int ToProtoData(kortex_driver::NotificationOptions input, Kinova::Api::Common::NotificationOptions *output)
{
	
	output->set_type((Kinova::Api::Common::NotificationType)input.type);
	output->set_rate_m_sec(input.rate_m_sec);
	output->set_threshold_value(input.threshold_value);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyHandle input, Kinova::Api::Common::SafetyHandle *output)
{
	
	output->set_identifier(input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::NotificationHandle input, Kinova::Api::Common::NotificationHandle *output)
{
	
	output->set_identifier(input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyNotification input, Kinova::Api::Common::SafetyNotification *output)
{
	 
	ToProtoData(input.safety_handle, output->mutable_safety_handle());
	output->set_value((Kinova::Api::Common::SafetyStatusValue)input.value); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::Timestamp input, Kinova::Api::Common::Timestamp *output)
{
	
	output->set_sec(input.sec);
	output->set_usec(input.usec);
	
	return 0;
}
int ToProtoData(kortex_driver::UserProfileHandle input, Kinova::Api::Common::UserProfileHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::Connection input, Kinova::Api::Common::Connection *output)
{
	 
	ToProtoData(input.user_handle, output->mutable_user_handle());
	output->set_connection_information(input.connection_information);
	output->set_connection_identifier(input.connection_identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::UARTConfiguration input, Kinova::Api::Common::UARTConfiguration *output)
{
	
	output->set_port_id(input.port_id);
	output->set_enabled(input.enabled);
	output->set_speed((Kinova::Api::Common::UARTSpeed)input.speed);
	output->set_word_length((Kinova::Api::Common::UARTWordLength)input.word_length);
	output->set_stop_bits((Kinova::Api::Common::UARTStopBits)input.stop_bits);
	output->set_parity((Kinova::Api::Common::UARTParity)input.parity);
	
	return 0;
}
int ToProtoData(kortex_driver::UARTDeviceIdentification input, Kinova::Api::Common::UARTDeviceIdentification *output)
{
	
	output->set_port_id(input.port_id);
	
	return 0;
}
int ToProtoData(kortex_driver::CountryCode input, Kinova::Api::Common::CountryCode *output)
{
	
	output->set_identifier((Kinova::Api::Common::CountryCodeIdentifier)input.identifier);
	
	return 0;
}
