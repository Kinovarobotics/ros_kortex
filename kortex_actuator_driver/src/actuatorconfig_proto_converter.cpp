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
 
#include "actuatorconfig_proto_converter.h"

#include "common_proto_converter.h"


int ToProtoData(kortex_actuator_driver::AxisPosition input, AxisPosition *output)
{
	output->set_position(input.position);

	return 0;
}
int ToProtoData(kortex_actuator_driver::AxisOffsets input, AxisOffsets *output)
{
	output->set_absolute_offset(input.absolute_offset);
	output->set_relative_offset(input.relative_offset);

	return 0;
}
int ToProtoData(kortex_actuator_driver::TorqueCalibration input, TorqueCalibration *output)
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
int ToProtoData(kortex_actuator_driver::TorqueOffset input, TorqueOffset *output)
{
	output->set_torque_offset(input.torque_offset);

	return 0;
}
int ToProtoData(kortex_actuator_driver::ControlModeInformation input, ControlModeInformation *output)
{
	output->set_control_mode((Kinova::Api::ActuatorConfig::ControlMode)input.control_mode);

	return 0;
}
int ToProtoData(kortex_actuator_driver::ControlLoop input, ControlLoop *output)
{
	output->set_control_loop(input.control_loop);

	return 0;
}
int ToProtoData(kortex_actuator_driver::LoopSelection input, LoopSelection *output)
{
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);

	return 0;
}
int ToProtoData(kortex_actuator_driver::VectorDriveParameters input, VectorDriveParameters *output)
{
	output->set_kpq(input.kpq);
	output->set_kiq(input.kiq);
	output->set_kpd(input.kpd);
	output->set_kid(input.kid);

	return 0;
}
int ToProtoData(kortex_actuator_driver::EncoderDerivativeParameters input, EncoderDerivativeParameters *output)
{
	output->set_max_window_width(input.max_window_width);
	output->set_min_encoder_tick_count(input.min_encoder_tick_count);

	return 0;
}
int ToProtoData(kortex_actuator_driver::ControlLoopParameters input, ControlLoopParameters *output)
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
	

	return 0;
}
int ToProtoData(kortex_actuator_driver::FrequencyResponse input, FrequencyResponse *output)
{
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_min_frequency(input.min_frequency);
	output->set_max_frequency(input.max_frequency);
	output->set_amplitude(input.amplitude);
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_actuator_driver::StepResponse input, StepResponse *output)
{
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_amplitude(input.amplitude);
	output->set_step_delay(input.step_delay);
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_actuator_driver::RampResponse input, RampResponse *output)
{
	output->set_loop_selection((Kinova::Api::ActuatorConfig::ControlLoopSelection)input.loop_selection);
	output->set_slope(input.slope);
	output->set_ramp_delay(input.ramp_delay);
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_actuator_driver::CustomDataSelection input, CustomDataSelection *output)
{  
	output->clear_channel();
	for(int i = 0; i < input.channel.size(); i++)
	{
		output->add_channel(CustomDataIndex(input.channel[i]));
	}

	return 0;
}
int ToProtoData(kortex_actuator_driver::CommandModeInformation input, CommandModeInformation *output)
{
	output->set_command_mode((Kinova::Api::ActuatorConfig::CommandMode)input.command_mode);

	return 0;
}
int ToProtoData(kortex_actuator_driver::Servoing input, Servoing *output)
{
	output->set_enabled(input.enabled);

	return 0;
}
int ToProtoData(kortex_actuator_driver::PositionCommand input, PositionCommand *output)
{
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_acceleration(input.acceleration);

	return 0;
}
