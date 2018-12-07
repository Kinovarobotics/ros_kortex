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
 
#include "visionconfig_ros_converter.h"

#include "common_ros_converter.h"


int ToRosData(SensorSettings input, kortex_vision_config_driver::SensorSettings &output)
{
	output.sensor = input.sensor();
	output.resolution = input.resolution();
	output.frame_rate = input.frame_rate();
	output.bit_rate = input.bit_rate();
	
	return 0;
}
int ToRosData(SensorIdentifier input, kortex_vision_config_driver::SensorIdentifier &output)
{
	output.sensor = input.sensor();
	
	return 0;
}
int ToRosData(OptionIdentifier input, kortex_vision_config_driver::OptionIdentifier &output)
{
	output.sensor = input.sensor();
	output.option = input.option();
	
	return 0;
}
int ToRosData(OptionValue input, kortex_vision_config_driver::OptionValue &output)
{
	output.sensor = input.sensor();
	output.option = input.option();
	output.value = input.value();
	
	return 0;
}
int ToRosData(OptionInformation input, kortex_vision_config_driver::OptionInformation &output)
{
	output.sensor = input.sensor();
	output.option = input.option();
	output.supported = input.supported();
	output.read_only = input.read_only();
	output.minimum = input.minimum();
	output.maximum = input.maximum();
	output.step = input.step();
	output.default_value = input.default_value();
	
	return 0;
}
int ToRosData(SensorFocusAction input, kortex_vision_config_driver::SensorFocusAction &output)
{
	output.sensor = input.sensor();
	output.focus_action = input.focus_action();
	
	return 0;
}
int ToRosData(VisionNotification input, kortex_vision_config_driver::VisionNotification &output)
{
	output.event = input.event();
	output.sensor = input.sensor();
	output.option = input.option();
	
	return 0;
}
int ToRosData(IntrinsicParameters input, kortex_vision_config_driver::IntrinsicParameters &output)
{
	output.width = input.width();
	output.height = input.height();
	output.principal_point_x = input.principal_point_x();
	output.principal_point_y = input.principal_point_y();
	output.focal_length_x = input.focal_length_x();
	output.focal_length_y = input.focal_length_y();
	
	return 0;
}
