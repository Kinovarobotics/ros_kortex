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
 
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"

int ToRosData(Kinova::Api::GripperCyclic::MessageId input, kortex_driver::GripperCyclic_MessageId &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::GripperCyclic::MotorCommand input, kortex_driver::MotorCommand &output)
{
	
	output.motor_id = input.motor_id();
	output.position = input.position();
	output.velocity = input.velocity();
	output.force = input.force();

	
	
	return 0;
}
int ToRosData(Kinova::Api::GripperCyclic::Command input, kortex_driver::GripperCyclic_Command &output)
{
	
	ToRosData(input.command_id(), output.command_id);
	output.flags = input.flags();
	output.motor_cmd.clear();
	for(int i = 0; i < input.motor_cmd_size(); i++)
	{
		decltype(output.motor_cmd)::value_type temp;
		ToRosData(input.motor_cmd(i), temp);
		output.motor_cmd.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::GripperCyclic::MotorFeedback input, kortex_driver::MotorFeedback &output)
{
	
	output.motor_id = input.motor_id();
	output.position = input.position();
	output.velocity = input.velocity();
	output.current_motor = input.current_motor();
	output.voltage = input.voltage();
	output.temperature_motor = input.temperature_motor();

	
	
	return 0;
}
int ToRosData(Kinova::Api::GripperCyclic::Feedback input, kortex_driver::GripperCyclic_Feedback &output)
{
	
	ToRosData(input.feedback_id(), output.feedback_id);
	output.status_flags = input.status_flags();
	output.fault_bank_a = input.fault_bank_a();
	output.fault_bank_b = input.fault_bank_b();
	output.warning_bank_a = input.warning_bank_a();
	output.warning_bank_b = input.warning_bank_b();
	output.motor.clear();
	for(int i = 0; i < input.motor_size(); i++)
	{
		decltype(output.motor)::value_type temp;
		ToRosData(input.motor(i), temp);
		output.motor.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::GripperCyclic::CustomDataUnit input, kortex_driver::CustomDataUnit &output)
{
	
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
int ToRosData(Kinova::Api::GripperCyclic::CustomData input, kortex_driver::GripperCyclic_CustomData &output)
{
	
	ToRosData(input.custom_data_id(), output.custom_data_id);
	ToRosData(input.gripper_custom_data(), output.gripper_custom_data);
	output.motor_custom_data.clear();
	for(int i = 0; i < input.motor_custom_data_size(); i++)
	{
		decltype(output.motor_custom_data)::value_type temp;
		ToRosData(input.motor_custom_data(i), temp);
		output.motor_custom_data.push_back(temp);
	}

	
	
	return 0;
}
