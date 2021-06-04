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
 
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"

int ToRosData(Kinova::Api::BaseCyclic::ActuatorCommand input, kortex_driver::ActuatorCommand &output)
{
	
	output.command_id = input.command_id();
	output.flags = input.flags();
	output.position = input.position();
	output.velocity = input.velocity();
	output.torque_joint = input.torque_joint();
	output.current_motor = input.current_motor();

	
	
	return 0;
}
int ToRosData(Kinova::Api::BaseCyclic::ActuatorFeedback input, kortex_driver::ActuatorFeedback &output)
{
	
	output.command_id = input.command_id();
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
int ToRosData(Kinova::Api::BaseCyclic::ActuatorCustomData input, kortex_driver::ActuatorCustomData &output)
{
	
	output.command_id = input.command_id();
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
int ToRosData(Kinova::Api::BaseCyclic::BaseFeedback input, kortex_driver::BaseFeedback &output)
{
	
	output.active_state_connection_identifier = input.active_state_connection_identifier();
	output.active_state = input.active_state();
	output.arm_voltage = input.arm_voltage();
	output.arm_current = input.arm_current();
	output.temperature_cpu = input.temperature_cpu();
	output.temperature_ambient = input.temperature_ambient();
	output.imu_acceleration_x = input.imu_acceleration_x();
	output.imu_acceleration_y = input.imu_acceleration_y();
	output.imu_acceleration_z = input.imu_acceleration_z();
	output.imu_angular_velocity_x = input.imu_angular_velocity_x();
	output.imu_angular_velocity_y = input.imu_angular_velocity_y();
	output.imu_angular_velocity_z = input.imu_angular_velocity_z();
	output.tool_pose_x = input.tool_pose_x();
	output.tool_pose_y = input.tool_pose_y();
	output.tool_pose_z = input.tool_pose_z();
	output.tool_pose_theta_x = input.tool_pose_theta_x();
	output.tool_pose_theta_y = input.tool_pose_theta_y();
	output.tool_pose_theta_z = input.tool_pose_theta_z();
	output.tool_twist_linear_x = input.tool_twist_linear_x();
	output.tool_twist_linear_y = input.tool_twist_linear_y();
	output.tool_twist_linear_z = input.tool_twist_linear_z();
	output.tool_twist_angular_x = input.tool_twist_angular_x();
	output.tool_twist_angular_y = input.tool_twist_angular_y();
	output.tool_twist_angular_z = input.tool_twist_angular_z();
	output.tool_external_wrench_force_x = input.tool_external_wrench_force_x();
	output.tool_external_wrench_force_y = input.tool_external_wrench_force_y();
	output.tool_external_wrench_force_z = input.tool_external_wrench_force_z();
	output.tool_external_wrench_torque_x = input.tool_external_wrench_torque_x();
	output.tool_external_wrench_torque_y = input.tool_external_wrench_torque_y();
	output.tool_external_wrench_torque_z = input.tool_external_wrench_torque_z();
	output.fault_bank_a = input.fault_bank_a();
	output.fault_bank_b = input.fault_bank_b();
	output.warning_bank_a = input.warning_bank_a();
	output.warning_bank_b = input.warning_bank_b();
	output.commanded_tool_pose_x = input.commanded_tool_pose_x();
	output.commanded_tool_pose_y = input.commanded_tool_pose_y();
	output.commanded_tool_pose_z = input.commanded_tool_pose_z();
	output.commanded_tool_pose_theta_x = input.commanded_tool_pose_theta_x();
	output.commanded_tool_pose_theta_y = input.commanded_tool_pose_theta_y();
	output.commanded_tool_pose_theta_z = input.commanded_tool_pose_theta_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::BaseCyclic::CustomData input, kortex_driver::BaseCyclic_CustomData &output)
{
	
	output.frame_id = input.frame_id();
	output.custom_data_0 = input.custom_data_0();
	output.custom_data_1 = input.custom_data_1();
	output.custom_data_2 = input.custom_data_2();
	output.custom_data_3 = input.custom_data_3();
	output.custom_data_4 = input.custom_data_4();
	output.custom_data_5 = input.custom_data_5();
	output.custom_data_6 = input.custom_data_6();
	output.custom_data_7 = input.custom_data_7();
	output.actuators_custom_data.clear();
	for(int i = 0; i < input.actuators_custom_data_size(); i++)
	{
		decltype(output.actuators_custom_data)::value_type temp;
		ToRosData(input.actuators_custom_data(i), temp);
		output.actuators_custom_data.push_back(temp);
	}
	ToRosData(input.interconnect_custom_data(), output.interconnect_custom_data);

	
	
	return 0;
}
int ToRosData(Kinova::Api::BaseCyclic::Command input, kortex_driver::BaseCyclic_Command &output)
{
	
	output.frame_id = input.frame_id();
	output.actuators.clear();
	for(int i = 0; i < input.actuators_size(); i++)
	{
		decltype(output.actuators)::value_type temp;
		ToRosData(input.actuators(i), temp);
		output.actuators.push_back(temp);
	}
	ToRosData(input.interconnect(), output.interconnect);

	
	
	return 0;
}
int ToRosData(Kinova::Api::BaseCyclic::Feedback input, kortex_driver::BaseCyclic_Feedback &output)
{
	
	output.frame_id = input.frame_id();
	ToRosData(input.base(), output.base);
	output.actuators.clear();
	for(int i = 0; i < input.actuators_size(); i++)
	{
		decltype(output.actuators)::value_type temp;
		ToRosData(input.actuators(i), temp);
		output.actuators.push_back(temp);
	}
	ToRosData(input.interconnect(), output.interconnect);

	
	
	return 0;
}
