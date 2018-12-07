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
 
#include "common_proto_converter.h"


int ToProtoData(kortex_actuator_driver::DeviceHandle input, DeviceHandle *output)
{
	output->set_device_type((Kinova::Api::Common::DeviceTypes)input.device_type);
	output->set_device_identifier(input.device_identifier);
	output->set_order(input.order);

	return 0;
}
int ToProtoData(kortex_actuator_driver::Empty input, Empty *output)
{

	return 0;
}
int ToProtoData(kortex_actuator_driver::NotificationOptions input, NotificationOptions *output)
{
	output->set_type((Kinova::Api::Common::NotificationType)input.type);
	output->set_rate_m_sec(input.rate_m_sec);
	output->set_threshold_value(input.threshold_value);

	return 0;
}
int ToProtoData(kortex_actuator_driver::SafetyHandle input, SafetyHandle *output)
{
	output->set_identifier(input.identifier);

	return 0;
}
int ToProtoData(kortex_actuator_driver::NotificationHandle input, NotificationHandle *output)
{
	output->set_identifier(input.identifier);

	return 0;
}
int ToProtoData(kortex_actuator_driver::SafetyNotification input, SafetyNotification *output)
{
	ToProtoData(input.safety_handle, output->mutable_safety_handle());
	output->set_value((Kinova::Api::Common::SafetyStatusValue)input.value);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_actuator_driver::Timestamp input, Timestamp *output)
{
	output->set_sec(input.sec);
	output->set_usec(input.usec);

	return 0;
}
int ToProtoData(kortex_actuator_driver::UserProfileHandle input, UserProfileHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_actuator_driver::Connection input, Connection *output)
{
	ToProtoData(input.user_handle, output->mutable_user_handle());
	output->set_connection_information(input.connection_information);
	output->set_connection_identifier(input.connection_identifier);

	return 0;
}
