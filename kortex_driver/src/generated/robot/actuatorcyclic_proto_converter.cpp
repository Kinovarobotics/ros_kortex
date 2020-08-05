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
 
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"

int ToProtoData(kortex_driver::ActuatorCyclic_MessageId input, Kinova::Api::ActuatorCyclic::MessageId *output)
{
	
	output->set_identifier(input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::ActuatorCyclic_Command input, Kinova::Api::ActuatorCyclic::Command *output)
{
	 
	ToProtoData(input.command_id, output->mutable_command_id());
	output->set_flags(input.flags);
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_torque_joint(input.torque_joint);
	output->set_current_motor(input.current_motor);
	
	return 0;
}
int ToProtoData(kortex_driver::ActuatorCyclic_Feedback input, Kinova::Api::ActuatorCyclic::Feedback *output)
{
	 
	ToProtoData(input.feedback_id, output->mutable_feedback_id());
	output->set_status_flags(input.status_flags);
	output->set_jitter_comm(input.jitter_comm);
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_torque(input.torque);
	output->set_current_motor(input.current_motor);
	output->set_voltage(input.voltage);
	output->set_temperature_motor(input.temperature_motor);
	output->set_temperature_core(input.temperature_core);
	output->set_fault_bank_a(input.fault_bank_a);
	output->set_fault_bank_b(input.fault_bank_b);
	output->set_warning_bank_a(input.warning_bank_a);
	output->set_warning_bank_b(input.warning_bank_b);
	
	return 0;
}
int ToProtoData(kortex_driver::ActuatorCyclic_CustomData input, Kinova::Api::ActuatorCyclic::CustomData *output)
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
	
	return 0;
}
