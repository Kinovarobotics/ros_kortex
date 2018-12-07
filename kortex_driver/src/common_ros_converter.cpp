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
 
#include "common_ros_converter.h"


int ToRosData(DeviceHandle input, kortex_driver::DeviceHandle &output)
{
	output.device_type = input.device_type();
	output.device_identifier = input.device_identifier();
	output.order = input.order();
	
	return 0;
}
int ToRosData(Empty input, kortex_driver::Empty &output)
{
	
	return 0;
}
int ToRosData(NotificationOptions input, kortex_driver::NotificationOptions &output)
{
	output.type = input.type();
	output.rate_m_sec = input.rate_m_sec();
	output.threshold_value = input.threshold_value();
	
	return 0;
}
int ToRosData(SafetyHandle input, kortex_driver::SafetyHandle &output)
{
	output.identifier = input.identifier();
	
	return 0;
}
int ToRosData(NotificationHandle input, kortex_driver::NotificationHandle &output)
{
	output.identifier = input.identifier();
	
	return 0;
}
int ToRosData(SafetyNotification input, kortex_driver::SafetyNotification &output)
{
	ToRosData(input.safety_handle(), output.safety_handle);
	output.value = input.value();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(Timestamp input, kortex_driver::Timestamp &output)
{
	output.sec = input.sec();
	output.usec = input.usec();
	
	return 0;
}
int ToRosData(UserProfileHandle input, kortex_driver::UserProfileHandle &output)
{
	output.identifier = input.identifier();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(Connection input, kortex_driver::Connection &output)
{
	ToRosData(input.user_handle(), output.user_handle);
	output.connection_information = input.connection_information();
	output.connection_identifier = input.connection_identifier();
	
	return 0;
}
