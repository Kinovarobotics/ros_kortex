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
 
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"

int ToRosData(Kinova::Api::ActuatorCyclic::MessageId input, kortex_driver::ActuatorCyclic_MessageId &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorCyclic::Command input, kortex_driver::ActuatorCyclic_Command &output)
{
	
	ToRosData(input.command_id(), output.command_id);
	output.flags = input.flags();
	output.position = input.position();
	output.velocity = input.velocity();
	output.torque_joint = input.torque_joint();
	output.current_motor = input.current_motor();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorCyclic::Feedback input, kortex_driver::ActuatorCyclic_Feedback &output)
{
	
	ToRosData(input.feedback_id(), output.feedback_id);
	output.status_flags = input.status_flags();
	output.jitter_comm = input.jitter_comm();
	output.position = input.position();
	output.velocity = input.velocity();
	output.torque = input.torque();
	output.current_motor = input.current_motor();
	output.voltage = input.voltage();
	output.temperature_motor = input.temperature_motor();
	output.temperature_core = input.temperature_core();
	output.fault_bank_a = input.fault_bank_a();
	output.fault_bank_b = input.fault_bank_b();
	output.warning_bank_a = input.warning_bank_a();
	output.warning_bank_b = input.warning_bank_b();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ActuatorCyclic::CustomData input, kortex_driver::ActuatorCyclic_CustomData &output)
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

	
	
	return 0;
}
