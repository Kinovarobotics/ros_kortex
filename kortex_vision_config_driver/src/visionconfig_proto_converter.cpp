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
 
#include "visionconfig_proto_converter.h"

#include "common_proto_converter.h"


int ToProtoData(kortex_vision_config_driver::SensorSettings input, SensorSettings *output)
{
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_resolution((Kinova::Api::VisionConfig::Resolution)input.resolution);
	output->set_frame_rate((Kinova::Api::VisionConfig::FrameRate)input.frame_rate);
	output->set_bit_rate((Kinova::Api::VisionConfig::BitRate)input.bit_rate);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::SensorIdentifier input, SensorIdentifier *output)
{
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::OptionIdentifier input, OptionIdentifier *output)
{
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::OptionValue input, OptionValue *output)
{
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::OptionInformation input, OptionInformation *output)
{
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);
	output->set_supported(input.supported);
	output->set_read_only(input.read_only);
	output->set_minimum(input.minimum);
	output->set_maximum(input.maximum);
	output->set_step(input.step);
	output->set_default_value(input.default_value);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::SensorFocusAction input, SensorFocusAction *output)
{
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_focus_action((Kinova::Api::VisionConfig::FocusAction)input.focus_action);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::VisionNotification input, VisionNotification *output)
{
	output->set_event((Kinova::Api::VisionConfig::VisionEvent)input.event);
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);

	return 0;
}
int ToProtoData(kortex_vision_config_driver::IntrinsicParameters input, IntrinsicParameters *output)
{
	output->set_width(input.width);
	output->set_height(input.height);
	output->set_principal_point_x(input.principal_point_x);
	output->set_principal_point_y(input.principal_point_y);
	output->set_focal_length_x(input.focal_length_x);
	output->set_focal_length_y(input.focal_length_y);

	return 0;
}
