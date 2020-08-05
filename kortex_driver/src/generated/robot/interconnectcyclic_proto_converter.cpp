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
 
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"

int ToProtoData(kortex_driver::InterconnectCyclic_MessageId input, Kinova::Api::InterconnectCyclic::MessageId *output)
{
	
	output->set_identifier(input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::InterconnectCyclic_Command input, Kinova::Api::InterconnectCyclic::Command *output)
{
	 
	ToProtoData(input.command_id, output->mutable_command_id());
	output->set_flags(input.flags);
	if(input.oneof_tool_command.gripper_command.size() > 0)
	{
		ToProtoData(input.oneof_tool_command.gripper_command[0], output->mutable_gripper_command());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::InterconnectCyclic_Feedback input, Kinova::Api::InterconnectCyclic::Feedback *output)
{
	 
	ToProtoData(input.feedback_id, output->mutable_feedback_id());
	output->set_status_flags(input.status_flags);
	output->set_jitter_comm(input.jitter_comm);
	output->set_imu_acceleration_x(input.imu_acceleration_x);
	output->set_imu_acceleration_y(input.imu_acceleration_y);
	output->set_imu_acceleration_z(input.imu_acceleration_z);
	output->set_imu_angular_velocity_x(input.imu_angular_velocity_x);
	output->set_imu_angular_velocity_y(input.imu_angular_velocity_y);
	output->set_imu_angular_velocity_z(input.imu_angular_velocity_z);
	output->set_voltage(input.voltage);
	output->set_temperature_core(input.temperature_core);
	output->set_fault_bank_a(input.fault_bank_a);
	output->set_fault_bank_b(input.fault_bank_b);
	output->set_warning_bank_a(input.warning_bank_a);
	output->set_warning_bank_b(input.warning_bank_b);
	if(input.oneof_tool_feedback.gripper_feedback.size() > 0)
	{
		ToProtoData(input.oneof_tool_feedback.gripper_feedback[0], output->mutable_gripper_feedback());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::InterconnectCyclic_CustomData input, Kinova::Api::InterconnectCyclic::CustomData *output)
{
	 
	ToProtoData(input.custom_data_id, output->mutable_custom_data_id());
	output->set_custom_data_0(input.custom_data_0);
	output->set_custom_data_1(input.custom_data_1);
	output->set_custom_data_2(input.custom_data_2);
	output->set_custom_data_3(input.custom_data_3);
	output->set_custom_data_4(input.custom_data_4);
	output->set_custom_data_5(input.custom_data_5);
	output->set_custom_data_6(input.custom_data_6);
	output->set_custom_data_7(input.custom_data_7);
	output->set_custom_data_8(input.custom_data_8);
	output->set_custom_data_9(input.custom_data_9);
	output->set_custom_data_10(input.custom_data_10);
	output->set_custom_data_11(input.custom_data_11);
	output->set_custom_data_12(input.custom_data_12);
	output->set_custom_data_13(input.custom_data_13);
	output->set_custom_data_14(input.custom_data_14);
	output->set_custom_data_15(input.custom_data_15);
	if(input.oneof_tool_customData.gripper_custom_data.size() > 0)
	{
		ToProtoData(input.oneof_tool_customData.gripper_custom_data[0], output->mutable_gripper_custom_data());
	}
	
	return 0;
}
