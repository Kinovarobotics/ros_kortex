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
 
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"

int ToProtoData(kortex_driver::SensorSettings input, Kinova::Api::VisionConfig::SensorSettings *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_resolution((Kinova::Api::VisionConfig::Resolution)input.resolution);
	output->set_frame_rate((Kinova::Api::VisionConfig::FrameRate)input.frame_rate);
	output->set_bit_rate((Kinova::Api::VisionConfig::BitRate)input.bit_rate);
	
	return 0;
}
int ToProtoData(kortex_driver::SensorIdentifier input, Kinova::Api::VisionConfig::SensorIdentifier *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	
	return 0;
}
int ToProtoData(kortex_driver::IntrinsicProfileIdentifier input, Kinova::Api::VisionConfig::IntrinsicProfileIdentifier *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_resolution((Kinova::Api::VisionConfig::Resolution)input.resolution);
	
	return 0;
}
int ToProtoData(kortex_driver::OptionIdentifier input, Kinova::Api::VisionConfig::OptionIdentifier *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);
	
	return 0;
}
int ToProtoData(kortex_driver::OptionValue input, Kinova::Api::VisionConfig::OptionValue *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::OptionInformation input, Kinova::Api::VisionConfig::OptionInformation *output)
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
int ToProtoData(kortex_driver::SensorFocusAction input, Kinova::Api::VisionConfig::SensorFocusAction *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_focus_action((Kinova::Api::VisionConfig::FocusAction)input.focus_action);
	if(input.oneof_action_parameters.focus_point.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.focus_point[0], output->mutable_focus_point());
	}
	if(input.oneof_action_parameters.manual_focus.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.manual_focus[0], output->mutable_manual_focus());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::FocusPoint input, Kinova::Api::VisionConfig::FocusPoint *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	
	return 0;
}
int ToProtoData(kortex_driver::ManualFocus input, Kinova::Api::VisionConfig::ManualFocus *output)
{
	
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::VisionNotification input, Kinova::Api::VisionConfig::VisionNotification *output)
{
	
	output->set_event((Kinova::Api::VisionConfig::VisionEvent)input.event);
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_option((Kinova::Api::VisionConfig::Option)input.option);
	
	return 0;
}
int ToProtoData(kortex_driver::IntrinsicParameters input, Kinova::Api::VisionConfig::IntrinsicParameters *output)
{
	
	output->set_sensor((Kinova::Api::VisionConfig::Sensor)input.sensor);
	output->set_resolution((Kinova::Api::VisionConfig::Resolution)input.resolution);
	output->set_principal_point_x(input.principal_point_x);
	output->set_principal_point_y(input.principal_point_y);
	output->set_focal_length_x(input.focal_length_x);
	output->set_focal_length_y(input.focal_length_y); 
	ToProtoData(input.distortion_coeffs, output->mutable_distortion_coeffs());
	
	return 0;
}
int ToProtoData(kortex_driver::DistortionCoefficients input, Kinova::Api::VisionConfig::DistortionCoefficients *output)
{
	
	output->set_k1(input.k1);
	output->set_k2(input.k2);
	output->set_k3(input.k3);
	output->set_p1(input.p1);
	output->set_p2(input.p2);
	
	return 0;
}
int ToProtoData(kortex_driver::ExtrinsicParameters input, Kinova::Api::VisionConfig::ExtrinsicParameters *output)
{
	 
	ToProtoData(input.rotation, output->mutable_rotation()); 
	ToProtoData(input.translation, output->mutable_translation());
	
	return 0;
}
int ToProtoData(kortex_driver::VisionConfig_RotationMatrix input, Kinova::Api::VisionConfig::RotationMatrix *output)
{
	 
	ToProtoData(input.row1, output->mutable_row1()); 
	ToProtoData(input.row2, output->mutable_row2()); 
	ToProtoData(input.row3, output->mutable_row3());
	
	return 0;
}
int ToProtoData(kortex_driver::VisionConfig_RotationMatrixRow input, Kinova::Api::VisionConfig::RotationMatrixRow *output)
{
	
	output->set_column1(input.column1);
	output->set_column2(input.column2);
	output->set_column3(input.column3);
	
	return 0;
}
int ToProtoData(kortex_driver::TranslationVector input, Kinova::Api::VisionConfig::TranslationVector *output)
{
	
	output->set_t_x(input.t_x);
	output->set_t_y(input.t_y);
	output->set_t_z(input.t_z);
	
	return 0;
}
