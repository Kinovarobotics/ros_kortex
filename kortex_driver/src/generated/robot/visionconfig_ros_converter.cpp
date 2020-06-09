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
 
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"

int ToRosData(Kinova::Api::VisionConfig::SensorSettings input, kortex_driver::SensorSettings &output)
{
	
	output.sensor = input.sensor();
	output.resolution = input.resolution();
	output.frame_rate = input.frame_rate();
	output.bit_rate = input.bit_rate();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::SensorIdentifier input, kortex_driver::SensorIdentifier &output)
{
	
	output.sensor = input.sensor();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::IntrinsicProfileIdentifier input, kortex_driver::IntrinsicProfileIdentifier &output)
{
	
	output.sensor = input.sensor();
	output.resolution = input.resolution();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::OptionIdentifier input, kortex_driver::OptionIdentifier &output)
{
	
	output.sensor = input.sensor();
	output.option = input.option();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::OptionValue input, kortex_driver::OptionValue &output)
{
	
	output.sensor = input.sensor();
	output.option = input.option();
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::OptionInformation input, kortex_driver::OptionInformation &output)
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
int ToRosData(Kinova::Api::VisionConfig::SensorFocusAction input, kortex_driver::SensorFocusAction &output)
{
	
	output.sensor = input.sensor();
	output.focus_action = input.focus_action();

	
	auto oneof_type_action_parameters = input.action_parameters_case();
	switch(oneof_type_action_parameters)
	{ 
	
		case Kinova::Api::VisionConfig::SensorFocusAction::kFocusPoint:
		{
			decltype(output.oneof_action_parameters.focus_point)::value_type temp;
			ToRosData(input.focus_point(), temp);
			output.oneof_action_parameters.focus_point.push_back(temp);
			break;
		} 
	
		case Kinova::Api::VisionConfig::SensorFocusAction::kManualFocus:
		{
			decltype(output.oneof_action_parameters.manual_focus)::value_type temp;
			ToRosData(input.manual_focus(), temp);
			output.oneof_action_parameters.manual_focus.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::FocusPoint input, kortex_driver::FocusPoint &output)
{
	
	output.x = input.x();
	output.y = input.y();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::ManualFocus input, kortex_driver::ManualFocus &output)
{
	
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::VisionNotification input, kortex_driver::VisionNotification &output)
{
	
	output.event = input.event();
	output.sensor = input.sensor();
	output.option = input.option();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::IntrinsicParameters input, kortex_driver::IntrinsicParameters &output)
{
	
	output.sensor = input.sensor();
	output.resolution = input.resolution();
	output.principal_point_x = input.principal_point_x();
	output.principal_point_y = input.principal_point_y();
	output.focal_length_x = input.focal_length_x();
	output.focal_length_y = input.focal_length_y();
	ToRosData(input.distortion_coeffs(), output.distortion_coeffs);

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::DistortionCoefficients input, kortex_driver::DistortionCoefficients &output)
{
	
	output.k1 = input.k1();
	output.k2 = input.k2();
	output.k3 = input.k3();
	output.p1 = input.p1();
	output.p2 = input.p2();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::ExtrinsicParameters input, kortex_driver::ExtrinsicParameters &output)
{
	
	ToRosData(input.rotation(), output.rotation);
	ToRosData(input.translation(), output.translation);

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::RotationMatrix input, kortex_driver::VisionConfig_RotationMatrix &output)
{
	
	ToRosData(input.row1(), output.row1);
	ToRosData(input.row2(), output.row2);
	ToRosData(input.row3(), output.row3);

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::RotationMatrixRow input, kortex_driver::VisionConfig_RotationMatrixRow &output)
{
	
	output.column1 = input.column1();
	output.column2 = input.column2();
	output.column3 = input.column3();

	
	
	return 0;
}
int ToRosData(Kinova::Api::VisionConfig::TranslationVector input, kortex_driver::TranslationVector &output)
{
	
	output.t_x = input.t_x();
	output.t_y = input.t_y();
	output.t_z = input.t_z();

	
	
	return 0;
}
