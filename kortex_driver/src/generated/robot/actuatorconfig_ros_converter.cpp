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
 
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"

int ToRosData(Kinova::Api::ActuatorConfig::AxisPosition input, kortex_driver::AxisPosition &output)
{
	
	output.position = input.position();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::AxisOffsets input, kortex_driver::AxisOffsets &output)
{
	
	output.absolute_offset = input.absolute_offset();
	output.relative_offset = input.relative_offset();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::TorqueCalibration input, kortex_driver::TorqueCalibration &output)
{
	
	output.global_gain = input.global_gain();
	output.global_offset = input.global_offset();
	output.gain.clear();
	for(int i = 0; i < input.gain_size(); i++)
	{
		output.gain.push_back(input.gain(i));
	}
	output.offset.clear();
	for(int i = 0; i < input.offset_size(); i++)
	{
		output.offset.push_back(input.offset(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::TorqueOffset input, kortex_driver::TorqueOffset &output)
{
	
	output.torque_offset = input.torque_offset();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::ControlModeInformation input, kortex_driver::ActuatorConfig_ControlModeInformation &output)
{
	
	output.control_mode = input.control_mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::ControlLoop input, kortex_driver::ControlLoop &output)
{
	
	output.control_loop = input.control_loop();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::LoopSelection input, kortex_driver::LoopSelection &output)
{
	
	output.loop_selection = input.loop_selection();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::VectorDriveParameters input, kortex_driver::VectorDriveParameters &output)
{
	
	output.kpq = input.kpq();
	output.kiq = input.kiq();
	output.kpd = input.kpd();
	output.kid = input.kid();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::EncoderDerivativeParameters input, kortex_driver::EncoderDerivativeParameters &output)
{
	
	output.max_window_width = input.max_window_width();
	output.min_angle = input.min_angle();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::ControlLoopParameters input, kortex_driver::ControlLoopParameters &output)
{
	
	output.loop_selection = input.loop_selection();
	output.error_saturation = input.error_saturation();
	output.output_saturation = input.output_saturation();
	output.kAz.clear();
	for(int i = 0; i < input.kaz_size(); i++)
	{
		output.kAz.push_back(input.kaz(i));
	}
	output.kBz.clear();
	for(int i = 0; i < input.kbz_size(); i++)
	{
		output.kBz.push_back(input.kbz(i));
	}
	output.error_dead_band = input.error_dead_band();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::FrequencyResponse input, kortex_driver::FrequencyResponse &output)
{
	
	output.loop_selection = input.loop_selection();
	output.min_frequency = input.min_frequency();
	output.max_frequency = input.max_frequency();
	output.amplitude = input.amplitude();
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::StepResponse input, kortex_driver::StepResponse &output)
{
	
	output.loop_selection = input.loop_selection();
	output.amplitude = input.amplitude();
	output.step_delay = input.step_delay();
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::RampResponse input, kortex_driver::RampResponse &output)
{
	
	output.loop_selection = input.loop_selection();
	output.slope = input.slope();
	output.ramp_delay = input.ramp_delay();
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::CustomDataSelection input, kortex_driver::CustomDataSelection &output)
{
	
	output.channel.clear();
	for(int i = 0; i < input.channel_size(); i++)
	{
		output.channel.push_back(input.channel(i));
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::CommandModeInformation input, kortex_driver::CommandModeInformation &output)
{
	
	output.command_mode = input.command_mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::Servoing input, kortex_driver::Servoing &output)
{
	
	output.enabled = input.enabled();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::PositionCommand input, kortex_driver::PositionCommand &output)
{
	
	output.position = input.position();
	output.velocity = input.velocity();
	output.acceleration = input.acceleration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation input, kortex_driver::CoggingFeedforwardModeInformation &output)
{
	
	output.cogging_feedforward_mode = input.cogging_feedforward_mode();

	
	
	return 0;
}
