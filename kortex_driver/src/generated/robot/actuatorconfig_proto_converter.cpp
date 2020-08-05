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
 
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"

int ToProtoData(kortex_driver::AxisPosition input, Kinova::Api::ActuatorConfig::AxisPosition *output)
{
	
	output->set_position(input.position);
	
	return 0;
}
int ToProtoData(kortex_driver::AxisOffsets input, Kinova::Api::ActuatorConfig::AxisOffsets *output)
{
	
	output->set_absolute_offset(input.absolute_offset);
	output->set_relative_offset(input.relative_offset);
	
	return 0;
}
int ToProtoData(kortex_driver::TorqueCalibration input, Kinova::Api::ActuatorConfig::TorqueCalibration *output)
{
	
	output->set_global_gain(input.global_gain);
	output->set_global_offset(input.global_offset);
	output->clear_gain();
	for(int i = 0; i < input.gain.size(); i++)
	{
		output->add_gain(input.gain[i]);
	}
	output->clear_offset();
	for(int i = 0; i < input.offset.size(); i++)
	{
		output->add_offset(input.offset[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::TorqueOffset input, Kinova::Api::ActuatorConfig::TorqueOffset *output)
{
	
	output->set_torque_offset(input.torque_offset);
	
	return 0;
}
int ToProtoData(kortex_driver::ActuatorConfig_ControlModeInformation input, Kinova::Api::ActuatorConfig::ControlModeInformation *output)
{
	
	output->set_control_mode((Kinova::Api::ActuatorConfig::ControlMode)input.control_mode);
	
	return 0;
}
int ToProtoData(kortex_driver::ControlLoop input, Kinova::Api::ActuatorConfig::ControlLoop *output)
{
	
	output->set_control_loop(input.control_loop);
	
	return 0;
}
int ToProtoData(kortex_driver::LoopSelection input, Kinova::Api::ActuatorConfig::LoopSelection *output)
{
	
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	
	return 0;
}
int ToProtoData(kortex_driver::VectorDriveParameters input, Kinova::Api::ActuatorConfig::VectorDriveParameters *output)
{
	
	output->set_kpq(input.kpq);
	output->set_kiq(input.kiq);
	output->set_kpd(input.kpd);
	output->set_kid(input.kid);
	
	return 0;
}
int ToProtoData(kortex_driver::EncoderDerivativeParameters input, Kinova::Api::ActuatorConfig::EncoderDerivativeParameters *output)
{
	
	output->set_max_window_width(input.max_window_width);
	output->set_min_angle(input.min_angle);
	
	return 0;
}
int ToProtoData(kortex_driver::ControlLoopParameters input, Kinova::Api::ActuatorConfig::ControlLoopParameters *output)
{
	
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_error_saturation(input.error_saturation);
	output->set_output_saturation(input.output_saturation);
	output->clear_kaz();
	for(int i = 0; i < input.kAz.size(); i++)
	{
		output->add_kaz(input.kAz[i]);
	}
	output->clear_kbz();
	for(int i = 0; i < input.kBz.size(); i++)
	{
		output->add_kbz(input.kBz[i]);
	}
	output->set_error_dead_band(input.error_dead_band);
	
	return 0;
}
int ToProtoData(kortex_driver::FrequencyResponse input, Kinova::Api::ActuatorConfig::FrequencyResponse *output)
{
	
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_min_frequency(input.min_frequency);
	output->set_max_frequency(input.max_frequency);
	output->set_amplitude(input.amplitude);
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::StepResponse input, Kinova::Api::ActuatorConfig::StepResponse *output)
{
	
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_amplitude(input.amplitude);
	output->set_step_delay(input.step_delay);
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::RampResponse input, Kinova::Api::ActuatorConfig::RampResponse *output)
{
	
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_slope(input.slope);
	output->set_ramp_delay(input.ramp_delay);
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::CustomDataSelection input, Kinova::Api::ActuatorConfig::CustomDataSelection *output)
{
	 
	output->clear_channel();
	for(int i = 0; i < input.channel.size(); i++)
	{
		output->add_channel((Kinova::Api::ActuatorConfig::CustomDataIndex)input.channel[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::CommandModeInformation input, Kinova::Api::ActuatorConfig::CommandModeInformation *output)
{
	
	output->set_command_mode((Kinova::Api::ActuatorConfig::CommandMode)input.command_mode);
	
	return 0;
}
int ToProtoData(kortex_driver::Servoing input, Kinova::Api::ActuatorConfig::Servoing *output)
{
	
	output->set_enabled(input.enabled);
	
	return 0;
}
int ToProtoData(kortex_driver::PositionCommand input, Kinova::Api::ActuatorConfig::PositionCommand *output)
{
	
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_acceleration(input.acceleration);
	
	return 0;
}
int ToProtoData(kortex_driver::CoggingFeedforwardModeInformation input, Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation *output)
{
	
	output->set_cogging_feedforward_mode((Kinova::Api::ActuatorConfig::CoggingFeedforwardMode)input.cogging_feedforward_mode);
	
	return 0;
}
