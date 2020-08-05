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
 
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"

int ToRosData(Kinova::Api::InterconnectCyclic::MessageId input, kortex_driver::InterconnectCyclic_MessageId &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectCyclic::Command input, kortex_driver::InterconnectCyclic_Command &output)
{
	
	ToRosData(input.command_id(), output.command_id);
	output.flags = input.flags();

	
	auto oneof_type_tool_command = input.tool_command_case();
	switch(oneof_type_tool_command)
	{ 
	
		case Kinova::Api::InterconnectCyclic::Command::kGripperCommand:
		{
			decltype(output.oneof_tool_command.gripper_command)::value_type temp;
			ToRosData(input.gripper_command(), temp);
			output.oneof_tool_command.gripper_command.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectCyclic::Feedback input, kortex_driver::InterconnectCyclic_Feedback &output)
{
	
	ToRosData(input.feedback_id(), output.feedback_id);
	output.status_flags = input.status_flags();
	output.jitter_comm = input.jitter_comm();
	output.imu_acceleration_x = input.imu_acceleration_x();
	output.imu_acceleration_y = input.imu_acceleration_y();
	output.imu_acceleration_z = input.imu_acceleration_z();
	output.imu_angular_velocity_x = input.imu_angular_velocity_x();
	output.imu_angular_velocity_y = input.imu_angular_velocity_y();
	output.imu_angular_velocity_z = input.imu_angular_velocity_z();
	output.voltage = input.voltage();
	output.temperature_core = input.temperature_core();
	output.fault_bank_a = input.fault_bank_a();
	output.fault_bank_b = input.fault_bank_b();
	output.warning_bank_a = input.warning_bank_a();
	output.warning_bank_b = input.warning_bank_b();

	
	auto oneof_type_tool_feedback = input.tool_feedback_case();
	switch(oneof_type_tool_feedback)
	{ 
	
		case Kinova::Api::InterconnectCyclic::Feedback::kGripperFeedback:
		{
			decltype(output.oneof_tool_feedback.gripper_feedback)::value_type temp;
			ToRosData(input.gripper_feedback(), temp);
			output.oneof_tool_feedback.gripper_feedback.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::InterconnectCyclic::CustomData input, kortex_driver::InterconnectCyclic_CustomData &output)
{
	
	ToRosData(input.custom_data_id(), output.custom_data_id);
	output.custom_data_0 = input.custom_data_0();
	output.custom_data_1 = input.custom_data_1();
	output.custom_data_2 = input.custom_data_2();
	output.custom_data_3 = input.custom_data_3();
	output.custom_data_4 = input.custom_data_4();
	output.custom_data_5 = input.custom_data_5();
	output.custom_data_6 = input.custom_data_6();
	output.custom_data_7 = input.custom_data_7();
	output.custom_data_8 = input.custom_data_8();
	output.custom_data_9 = input.custom_data_9();
	output.custom_data_10 = input.custom_data_10();
	output.custom_data_11 = input.custom_data_11();
	output.custom_data_12 = input.custom_data_12();
	output.custom_data_13 = input.custom_data_13();
	output.custom_data_14 = input.custom_data_14();
	output.custom_data_15 = input.custom_data_15();

	
	auto oneof_type_tool_customData = input.tool_customData_case();
	switch(oneof_type_tool_customData)
	{ 
	
		case Kinova::Api::InterconnectCyclic::CustomData::kGripperCustomData:
		{
			decltype(output.oneof_tool_customData.gripper_custom_data)::value_type temp;
			ToRosData(input.gripper_custom_data(), temp);
			output.oneof_tool_customData.gripper_custom_data.push_back(temp);
			break;
		}}
	
	return 0;
}
