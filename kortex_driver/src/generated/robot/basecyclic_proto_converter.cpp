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
 
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"

int ToProtoData(kortex_driver::ActuatorCommand input, Kinova::Api::BaseCyclic::ActuatorCommand *output)
{
	
	output->set_command_id(input.command_id);
	output->set_flags(input.flags);
	output->set_position(input.position);
	output->set_velocity(input.velocity);
	output->set_torque_joint(input.torque_joint);
	output->set_current_motor(input.current_motor);
	
	return 0;
}
int ToProtoData(kortex_driver::ActuatorFeedback input, Kinova::Api::BaseCyclic::ActuatorFeedback *output)
{
	
	output->set_command_id(input.command_id);
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
int ToProtoData(kortex_driver::ActuatorCustomData input, Kinova::Api::BaseCyclic::ActuatorCustomData *output)
{
	
	output->set_command_id(input.command_id);
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
int ToProtoData(kortex_driver::BaseFeedback input, Kinova::Api::BaseCyclic::BaseFeedback *output)
{
	
	output->set_active_state_connection_identifier(input.active_state_connection_identifier);
	output->set_active_state((Kinova::Api::Common::ArmState)input.active_state);
	output->set_arm_voltage(input.arm_voltage);
	output->set_arm_current(input.arm_current);
	output->set_temperature_cpu(input.temperature_cpu);
	output->set_temperature_ambient(input.temperature_ambient);
	output->set_imu_acceleration_x(input.imu_acceleration_x);
	output->set_imu_acceleration_y(input.imu_acceleration_y);
	output->set_imu_acceleration_z(input.imu_acceleration_z);
	output->set_imu_angular_velocity_x(input.imu_angular_velocity_x);
	output->set_imu_angular_velocity_y(input.imu_angular_velocity_y);
	output->set_imu_angular_velocity_z(input.imu_angular_velocity_z);
	output->set_tool_pose_x(input.tool_pose_x);
	output->set_tool_pose_y(input.tool_pose_y);
	output->set_tool_pose_z(input.tool_pose_z);
	output->set_tool_pose_theta_x(input.tool_pose_theta_x);
	output->set_tool_pose_theta_y(input.tool_pose_theta_y);
	output->set_tool_pose_theta_z(input.tool_pose_theta_z);
	output->set_tool_twist_linear_x(input.tool_twist_linear_x);
	output->set_tool_twist_linear_y(input.tool_twist_linear_y);
	output->set_tool_twist_linear_z(input.tool_twist_linear_z);
	output->set_tool_twist_angular_x(input.tool_twist_angular_x);
	output->set_tool_twist_angular_y(input.tool_twist_angular_y);
	output->set_tool_twist_angular_z(input.tool_twist_angular_z);
	output->set_tool_external_wrench_force_x(input.tool_external_wrench_force_x);
	output->set_tool_external_wrench_force_y(input.tool_external_wrench_force_y);
	output->set_tool_external_wrench_force_z(input.tool_external_wrench_force_z);
	output->set_tool_external_wrench_torque_x(input.tool_external_wrench_torque_x);
	output->set_tool_external_wrench_torque_y(input.tool_external_wrench_torque_y);
	output->set_tool_external_wrench_torque_z(input.tool_external_wrench_torque_z);
	output->set_fault_bank_a(input.fault_bank_a);
	output->set_fault_bank_b(input.fault_bank_b);
	output->set_warning_bank_a(input.warning_bank_a);
	output->set_warning_bank_b(input.warning_bank_b);
	output->set_commanded_tool_pose_x(input.commanded_tool_pose_x);
	output->set_commanded_tool_pose_y(input.commanded_tool_pose_y);
	output->set_commanded_tool_pose_z(input.commanded_tool_pose_z);
	output->set_commanded_tool_pose_theta_x(input.commanded_tool_pose_theta_x);
	output->set_commanded_tool_pose_theta_y(input.commanded_tool_pose_theta_y);
	output->set_commanded_tool_pose_theta_z(input.commanded_tool_pose_theta_z);
	
	return 0;
}
int ToProtoData(kortex_driver::BaseCyclic_CustomData input, Kinova::Api::BaseCyclic::CustomData *output)
{
	
	output->set_frame_id(input.frame_id);
	output->set_custom_data_0(input.custom_data_0);
	output->set_custom_data_1(input.custom_data_1);
	output->set_custom_data_2(input.custom_data_2);
	output->set_custom_data_3(input.custom_data_3);
	output->set_custom_data_4(input.custom_data_4);
	output->set_custom_data_5(input.custom_data_5);
	output->set_custom_data_6(input.custom_data_6);
	output->set_custom_data_7(input.custom_data_7); 
	output->clear_actuators_custom_data();
	for(int i = 0; i < input.actuators_custom_data.size(); i++)
	{
		ToProtoData(input.actuators_custom_data[i], output->add_actuators_custom_data());
	} 
	ToProtoData(input.interconnect_custom_data, output->mutable_interconnect_custom_data());
	
	return 0;
}
int ToProtoData(kortex_driver::BaseCyclic_Command input, Kinova::Api::BaseCyclic::Command *output)
{
	
	output->set_frame_id(input.frame_id); 
	output->clear_actuators();
	for(int i = 0; i < input.actuators.size(); i++)
	{
		ToProtoData(input.actuators[i], output->add_actuators());
	} 
	ToProtoData(input.interconnect, output->mutable_interconnect());
	
	return 0;
}
int ToProtoData(kortex_driver::BaseCyclic_Feedback input, Kinova::Api::BaseCyclic::Feedback *output)
{
	
	output->set_frame_id(input.frame_id); 
	ToProtoData(input.base, output->mutable_base()); 
	output->clear_actuators();
	for(int i = 0; i < input.actuators.size(); i++)
	{
		ToProtoData(input.actuators[i], output->add_actuators());
	} 
	ToProtoData(input.interconnect, output->mutable_interconnect());
	
	return 0;
}
