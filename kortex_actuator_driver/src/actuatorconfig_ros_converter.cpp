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
 
#include "actuatorconfig_ros_converter.h"

#include "common_ros_converter.h"


int ToRosData(AxisPosition input, kortex_actuator_driver::AxisPosition &output)
{
	output.position = input.position();
	
	return 0;
}
int ToRosData(AxisOffsets input, kortex_actuator_driver::AxisOffsets &output)
{
	output.absolute_offset = input.absolute_offset();
	output.relative_offset = input.relative_offset();
	
	return 0;
}
int ToRosData(TorqueCalibration input, kortex_actuator_driver::TorqueCalibration &output)
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
int ToRosData(TorqueOffset input, kortex_actuator_driver::TorqueOffset &output)
{
	output.torque_offset = input.torque_offset();
	
	return 0;
}
int ToRosData(ControlModeInformation input, kortex_actuator_driver::ControlModeInformation &output)
{
	output.control_mode = input.control_mode();
	
	return 0;
}
int ToRosData(ControlLoop input, kortex_actuator_driver::ControlLoop &output)
{
	output.control_loop = input.control_loop();
	
	return 0;
}
int ToRosData(LoopSelection input, kortex_actuator_driver::LoopSelection &output)
{
	output.loop_selection = input.loop_selection();
	
	return 0;
}
int ToRosData(VectorDriveParameters input, kortex_actuator_driver::VectorDriveParameters &output)
{
	output.kpq = input.kpq();
	output.kiq = input.kiq();
	output.kpd = input.kpd();
	output.kid = input.kid();
	
	return 0;
}
int ToRosData(EncoderDerivativeParameters input, kortex_actuator_driver::EncoderDerivativeParameters &output)
{
	output.max_window_width = input.max_window_width();
	output.min_encoder_tick_count = input.min_encoder_tick_count();
	
	return 0;
}
int ToRosData(ControlLoopParameters input, kortex_actuator_driver::ControlLoopParameters &output)
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
	
	return 0;
}
int ToRosData(FrequencyResponse input, kortex_actuator_driver::FrequencyResponse &output)
{
	output.loop_selection = input.loop_selection();
	output.min_frequency = input.min_frequency();
	output.max_frequency = input.max_frequency();
	output.amplitude = input.amplitude();
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(StepResponse input, kortex_actuator_driver::StepResponse &output)
{
	output.loop_selection = input.loop_selection();
	output.amplitude = input.amplitude();
	output.step_delay = input.step_delay();
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(RampResponse input, kortex_actuator_driver::RampResponse &output)
{
	output.loop_selection = input.loop_selection();
	output.slope = input.slope();
	output.ramp_delay = input.ramp_delay();
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(CustomDataSelection input, kortex_actuator_driver::CustomDataSelection &output)
{ 
	
	output.channel.clear();
	for(int i = 0; i < input.channel_size(); i++)
	{
		output.channel.push_back(input.channel(i));
	}
	
	return 0;
}
int ToRosData(CommandModeInformation input, kortex_actuator_driver::CommandModeInformation &output)
{
	output.command_mode = input.command_mode();
	
	return 0;
}
int ToRosData(Servoing input, kortex_actuator_driver::Servoing &output)
{
	output.enabled = input.enabled();
	
	return 0;
}
int ToRosData(PositionCommand input, kortex_actuator_driver::PositionCommand &output)
{
	output.position = input.position();
	output.velocity = input.velocity();
	output.acceleration = input.acceleration();
	
	return 0;
}
