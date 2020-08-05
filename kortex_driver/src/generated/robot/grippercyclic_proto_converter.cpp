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
 
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"

int ToProtoData(kortex_driver::GripperCyclic_MessageId input, Kinova::Api::GripperCyclic::MessageId *output)
{
	
	output->set_identifier(input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::MotorCommand input, Kinova::Api::GripperCyclic::MotorCommand *output)
{
	
	output->set_motor_id(input.motor_id);
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_force(input.force);
	
	return 0;
}
int ToProtoData(kortex_driver::GripperCyclic_Command input, Kinova::Api::GripperCyclic::Command *output)
{
	 
	ToProtoData(input.command_id, output->mutable_command_id());
	output->set_flags(input.flags); 
	output->clear_motor_cmd();
	for(int i = 0; i < input.motor_cmd.size(); i++)
	{
		ToProtoData(input.motor_cmd[i], output->add_motor_cmd());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MotorFeedback input, Kinova::Api::GripperCyclic::MotorFeedback *output)
{
	
	output->set_motor_id(input.motor_id);
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_current_motor(input.current_motor);
	output->set_voltage(input.voltage);
	output->set_temperature_motor(input.temperature_motor);
	
	return 0;
}
int ToProtoData(kortex_driver::GripperCyclic_Feedback input, Kinova::Api::GripperCyclic::Feedback *output)
{
	 
	ToProtoData(input.feedback_id, output->mutable_feedback_id());
	output->set_status_flags(input.status_flags);
	output->set_fault_bank_a(input.fault_bank_a);
	output->set_fault_bank_b(input.fault_bank_b);
	output->set_warning_bank_a(input.warning_bank_a);
	output->set_warning_bank_b(input.warning_bank_b); 
	output->clear_motor();
	for(int i = 0; i < input.motor.size(); i++)
	{
		ToProtoData(input.motor[i], output->add_motor());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::CustomDataUnit input, Kinova::Api::GripperCyclic::CustomDataUnit *output)
{
	
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
	
	return 0;
}
int ToProtoData(kortex_driver::GripperCyclic_CustomData input, Kinova::Api::GripperCyclic::CustomData *output)
{
	 
	ToProtoData(input.custom_data_id, output->mutable_custom_data_id()); 
	ToProtoData(input.gripper_custom_data, output->mutable_gripper_custom_data()); 
	output->clear_motor_custom_data();
	for(int i = 0; i < input.motor_custom_data.size(); i++)
	{
		ToProtoData(input.motor_custom_data[i], output->add_motor_custom_data());
	}
	
	return 0;
}
