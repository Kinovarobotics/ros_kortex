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
 
#include "kortex_driver/generated/robot/base_proto_converter.h"

int ToProtoData(kortex_driver::GpioConfigurationList input, Kinova::Api::Base::GpioConfigurationList *output)
{
	 
	output->clear_port_configurations();
	for(int i = 0; i < input.port_configurations.size(); i++)
	{
		ToProtoData(input.port_configurations[i], output->add_port_configurations());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Base_GpioConfiguration input, Kinova::Api::Base::GpioConfiguration *output)
{
	
	output->set_port_number(input.port_number); 
	output->clear_pin_configurations();
	for(int i = 0; i < input.pin_configurations.size(); i++)
	{
		ToProtoData(input.pin_configurations[i], output->add_pin_configurations());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::GpioPinConfiguration input, Kinova::Api::Base::GpioPinConfiguration *output)
{
	
	output->set_pin_id(input.pin_id);
	output->set_pin_property((Kinova::Api::Base::GpioPinPropertyFlags)input.pin_property);
	output->set_output_enable(input.output_enable);
	output->set_default_output_value(input.default_output_value);
	
	return 0;
}
int ToProtoData(kortex_driver::FullUserProfile input, Kinova::Api::Base::FullUserProfile *output)
{
	 
	ToProtoData(input.user_profile, output->mutable_user_profile());
	output->set_password(input.password);
	
	return 0;
}
int ToProtoData(kortex_driver::UserProfile input, Kinova::Api::Base::UserProfile *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_username(input.username);
	output->set_firstname(input.firstname);
	output->set_lastname(input.lastname);
	output->set_application_data(input.application_data);
	
	return 0;
}
int ToProtoData(kortex_driver::UserProfileList input, Kinova::Api::Base::UserProfileList *output)
{
	 
	output->clear_user_profiles();
	for(int i = 0; i < input.user_profiles.size(); i++)
	{
		ToProtoData(input.user_profiles[i], output->add_user_profiles());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::UserList input, Kinova::Api::Base::UserList *output)
{
	 
	output->clear_user_handles();
	for(int i = 0; i < input.user_handles.size(); i++)
	{
		ToProtoData(input.user_handles[i], output->add_user_handles());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::PasswordChange input, Kinova::Api::Base::PasswordChange *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_old_password(input.old_password);
	output->set_new_password(input.new_password);
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceHandle input, Kinova::Api::Base::SequenceHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::AdvancedSequenceHandle input, Kinova::Api::Base::AdvancedSequenceHandle *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_in_loop(input.in_loop);
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTaskHandle input, Kinova::Api::Base::SequenceTaskHandle *output)
{
	 
	ToProtoData(input.sequence_handle, output->mutable_sequence_handle());
	output->set_task_index(input.task_index);
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTask input, Kinova::Api::Base::SequenceTask *output)
{
	
	output->set_group_identifier(input.group_identifier); 
	ToProtoData(input.action, output->mutable_action());
	output->set_application_data(input.application_data);
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTasks input, Kinova::Api::Base::SequenceTasks *output)
{
	 
	output->clear_sequence_tasks();
	for(int i = 0; i < input.sequence_tasks.size(); i++)
	{
		ToProtoData(input.sequence_tasks[i], output->add_sequence_tasks());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTasksConfiguration input, Kinova::Api::Base::SequenceTasksConfiguration *output)
{
	 
	ToProtoData(input.sequence_task_handle, output->mutable_sequence_task_handle()); 
	output->clear_sequence_tasks();
	for(int i = 0; i < input.sequence_tasks.size(); i++)
	{
		ToProtoData(input.sequence_tasks[i], output->add_sequence_tasks());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTaskConfiguration input, Kinova::Api::Base::SequenceTaskConfiguration *output)
{
	 
	ToProtoData(input.sequence_task_handle, output->mutable_sequence_task_handle()); 
	ToProtoData(input.sequence_task, output->mutable_sequence_task());
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTasksRange input, Kinova::Api::Base::SequenceTasksRange *output)
{
	
	output->set_first_task_index(input.first_task_index);
	output->set_second_task_index(input.second_task_index);
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceTasksPair input, Kinova::Api::Base::SequenceTasksPair *output)
{
	 
	ToProtoData(input.sequence_handle, output->mutable_sequence_handle());
	output->set_first_task_index(input.first_task_index);
	output->set_second_task_index(input.second_task_index);
	
	return 0;
}
int ToProtoData(kortex_driver::Sequence input, Kinova::Api::Base::Sequence *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name);
	output->set_application_data(input.application_data); 
	output->clear_tasks();
	for(int i = 0; i < input.tasks.size(); i++)
	{
		ToProtoData(input.tasks[i], output->add_tasks());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceList input, Kinova::Api::Base::SequenceList *output)
{
	 
	output->clear_sequence_list();
	for(int i = 0; i < input.sequence_list.size(); i++)
	{
		ToProtoData(input.sequence_list[i], output->add_sequence_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::AppendActionInformation input, Kinova::Api::Base::AppendActionInformation *output)
{
	 
	ToProtoData(input.sequence_handle, output->mutable_sequence_handle()); 
	ToProtoData(input.action, output->mutable_action());
	
	return 0;
}
int ToProtoData(kortex_driver::ActionHandle input, Kinova::Api::Base::ActionHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_action_type((Kinova::Api::Base::ActionType)input.action_type);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::RequestedActionType input, Kinova::Api::Base::RequestedActionType *output)
{
	
	output->set_action_type((Kinova::Api::Base::ActionType)input.action_type);
	
	return 0;
}
int ToProtoData(kortex_driver::Action input, Kinova::Api::Base::Action *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name);
	output->set_application_data(input.application_data);
	if(input.oneof_action_parameters.send_twist_command.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.send_twist_command[0], output->mutable_send_twist_command());
	}
	if(input.oneof_action_parameters.send_wrench_command.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.send_wrench_command[0], output->mutable_send_wrench_command());
	}
	if(input.oneof_action_parameters.send_joint_speeds.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.send_joint_speeds[0], output->mutable_send_joint_speeds());
	}
	if(input.oneof_action_parameters.reach_pose.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.reach_pose[0], output->mutable_reach_pose());
	}
	if(input.oneof_action_parameters.reach_joint_angles.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.reach_joint_angles[0], output->mutable_reach_joint_angles());
	}
	if(input.oneof_action_parameters.toggle_admittance_mode.size() > 0)
	{
		output->set_toggle_admittance_mode((Kinova::Api::Base::AdmittanceMode)input.oneof_action_parameters.toggle_admittance_mode[0]);
	}
	if(input.oneof_action_parameters.snapshot.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.snapshot[0], output->mutable_snapshot());
	}
	if(input.oneof_action_parameters.switch_control_mapping.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.switch_control_mapping[0], output->mutable_switch_control_mapping());
	}
	if(input.oneof_action_parameters.navigate_joints.size() > 0)
	{
		output->set_navigate_joints((Kinova::Api::Base::JointNavigationDirection)input.oneof_action_parameters.navigate_joints[0]);
	}
	if(input.oneof_action_parameters.navigate_mappings.size() > 0)
	{
		output->set_navigate_mappings((Kinova::Api::Base::NavigationDirection)input.oneof_action_parameters.navigate_mappings[0]);
	}
	if(input.oneof_action_parameters.change_twist.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.change_twist[0], output->mutable_change_twist());
	}
	if(input.oneof_action_parameters.change_joint_speeds.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.change_joint_speeds[0], output->mutable_change_joint_speeds());
	}
	if(input.oneof_action_parameters.change_wrench.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.change_wrench[0], output->mutable_change_wrench());
	}
	if(input.oneof_action_parameters.apply_emergency_stop.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.apply_emergency_stop[0], output->mutable_apply_emergency_stop());
	}
	if(input.oneof_action_parameters.clear_faults.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.clear_faults[0], output->mutable_clear_faults());
	}
	if(input.oneof_action_parameters.delay.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.delay[0], output->mutable_delay());
	}
	if(input.oneof_action_parameters.execute_action.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.execute_action[0], output->mutable_execute_action());
	}
	if(input.oneof_action_parameters.send_gripper_command.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.send_gripper_command[0], output->mutable_send_gripper_command());
	}
	if(input.oneof_action_parameters.send_gpio_command.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.send_gpio_command[0], output->mutable_send_gpio_command());
	}
	if(input.oneof_action_parameters.stop_action.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.stop_action[0], output->mutable_stop_action());
	}
	if(input.oneof_action_parameters.play_pre_computed_trajectory.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.play_pre_computed_trajectory[0], output->mutable_play_pre_computed_trajectory());
	}
	if(input.oneof_action_parameters.execute_sequence.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.execute_sequence[0], output->mutable_execute_sequence());
	}
	if(input.oneof_action_parameters.execute_waypoint_list.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.execute_waypoint_list[0], output->mutable_execute_waypoint_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Snapshot input, Kinova::Api::Base::Snapshot *output)
{
	
	output->set_snapshot_type((Kinova::Api::Base::SnapshotType)input.snapshot_type);
	
	return 0;
}
int ToProtoData(kortex_driver::SwitchControlMapping input, Kinova::Api::Base::SwitchControlMapping *output)
{
	
	output->set_controller_identifier(input.controller_identifier); 
	ToProtoData(input.map_group_handle, output->mutable_map_group_handle()); 
	ToProtoData(input.map_handle, output->mutable_map_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::ChangeTwist input, Kinova::Api::Base::ChangeTwist *output)
{
	
	output->set_linear(input.linear);
	output->set_angular(input.angular);
	
	return 0;
}
int ToProtoData(kortex_driver::ChangeJointSpeeds input, Kinova::Api::Base::ChangeJointSpeeds *output)
{
	 
	ToProtoData(input.joint_speeds, output->mutable_joint_speeds());
	
	return 0;
}
int ToProtoData(kortex_driver::ChangeWrench input, Kinova::Api::Base::ChangeWrench *output)
{
	
	output->set_force(input.force);
	output->set_torque(input.torque);
	
	return 0;
}
int ToProtoData(kortex_driver::EmergencyStop input, Kinova::Api::Base::EmergencyStop *output)
{
	
	
	return 0;
}
int ToProtoData(kortex_driver::Faults input, Kinova::Api::Base::Faults *output)
{
	
	
	return 0;
}
int ToProtoData(kortex_driver::Delay input, Kinova::Api::Base::Delay *output)
{
	
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::Base_Stop input, Kinova::Api::Base::Stop *output)
{
	
	
	return 0;
}
int ToProtoData(kortex_driver::ActionList input, Kinova::Api::Base::ActionList *output)
{
	 
	output->clear_action_list();
	for(int i = 0; i < input.action_list.size(); i++)
	{
		ToProtoData(input.action_list[i], output->add_action_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Timeout input, Kinova::Api::Base::Timeout *output)
{
	
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::Ssid input, Kinova::Api::Base::Ssid *output)
{
	
	output->set_identifier(input.identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::CommunicationInterfaceConfiguration input, Kinova::Api::Base::CommunicationInterfaceConfiguration *output)
{
	
	output->set_type((Kinova::Api::Base::NetworkType)input.type);
	output->set_enable(input.enable);
	
	return 0;
}
int ToProtoData(kortex_driver::NetworkHandle input, Kinova::Api::Base::NetworkHandle *output)
{
	
	output->set_type((Kinova::Api::Base::NetworkType)input.type);
	
	return 0;
}
int ToProtoData(kortex_driver::IPv4Configuration input, Kinova::Api::Base::IPv4Configuration *output)
{
	
	output->set_ip_address(input.ip_address);
	output->set_subnet_mask(input.subnet_mask);
	output->set_default_gateway(input.default_gateway);
	output->set_dhcp_enabled(input.dhcp_enabled);
	
	return 0;
}
int ToProtoData(kortex_driver::IPv4Information input, Kinova::Api::Base::IPv4Information *output)
{
	
	output->set_ip_address(input.ip_address);
	output->set_subnet_mask(input.subnet_mask);
	output->set_default_gateway(input.default_gateway);
	
	return 0;
}
int ToProtoData(kortex_driver::FullIPv4Configuration input, Kinova::Api::Base::FullIPv4Configuration *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle()); 
	ToProtoData(input.ipv4_configuration, output->mutable_ipv4_configuration());
	
	return 0;
}
int ToProtoData(kortex_driver::WifiInformation input, Kinova::Api::Base::WifiInformation *output)
{
	 
	ToProtoData(input.ssid, output->mutable_ssid());
	output->set_security_type(input.security_type);
	output->set_encryption_type(input.encryption_type);
	output->set_signal_quality((Kinova::Api::Base::SignalQuality)input.signal_quality);
	output->set_signal_strength(input.signal_strength);
	output->set_frequency(input.frequency);
	output->set_channel(input.channel);
	
	return 0;
}
int ToProtoData(kortex_driver::WifiInformationList input, Kinova::Api::Base::WifiInformationList *output)
{
	 
	output->clear_wifi_information_list();
	for(int i = 0; i < input.wifi_information_list.size(); i++)
	{
		ToProtoData(input.wifi_information_list[i], output->add_wifi_information_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::WifiConfiguration input, Kinova::Api::Base::WifiConfiguration *output)
{
	 
	ToProtoData(input.ssid, output->mutable_ssid());
	output->set_security_key(input.security_key);
	output->set_connect_automatically(input.connect_automatically);
	
	return 0;
}
int ToProtoData(kortex_driver::WifiConfigurationList input, Kinova::Api::Base::WifiConfigurationList *output)
{
	 
	output->clear_wifi_configuration_list();
	for(int i = 0; i < input.wifi_configuration_list.size(); i++)
	{
		ToProtoData(input.wifi_configuration_list[i], output->add_wifi_configuration_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneHandle input, Kinova::Api::Base::ProtectionZoneHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::Base_RotationMatrixRow input, Kinova::Api::Base::RotationMatrixRow *output)
{
	
	output->set_column1(input.column1);
	output->set_column2(input.column2);
	output->set_column3(input.column3);
	
	return 0;
}
int ToProtoData(kortex_driver::Base_RotationMatrix input, Kinova::Api::Base::RotationMatrix *output)
{
	 
	ToProtoData(input.row1, output->mutable_row1()); 
	ToProtoData(input.row2, output->mutable_row2()); 
	ToProtoData(input.row3, output->mutable_row3());
	
	return 0;
}
int ToProtoData(kortex_driver::Point input, Kinova::Api::Base::Point *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	
	return 0;
}
int ToProtoData(kortex_driver::ZoneShape input, Kinova::Api::Base::ZoneShape *output)
{
	
	output->set_shape_type((Kinova::Api::Base::ShapeType)input.shape_type); 
	ToProtoData(input.origin, output->mutable_origin()); 
	ToProtoData(input.orientation, output->mutable_orientation());
	output->clear_dimensions();
	for(int i = 0; i < input.dimensions.size(); i++)
	{
		output->add_dimensions(input.dimensions[i]);
	}
	output->set_envelope_thickness(input.envelope_thickness);
	
	return 0;
}
int ToProtoData(kortex_driver::ProtectionZone input, Kinova::Api::Base::ProtectionZone *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name);
	output->set_application_data(input.application_data);
	output->set_is_enabled(input.is_enabled); 
	ToProtoData(input.shape, output->mutable_shape()); 
	output->clear_limitations();
	for(int i = 0; i < input.limitations.size(); i++)
	{
		ToProtoData(input.limitations[i], output->add_limitations());
	} 
	output->clear_envelope_limitations();
	for(int i = 0; i < input.envelope_limitations.size(); i++)
	{
		ToProtoData(input.envelope_limitations[i], output->add_envelope_limitations());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneList input, Kinova::Api::Base::ProtectionZoneList *output)
{
	 
	output->clear_protection_zones();
	for(int i = 0; i < input.protection_zones.size(); i++)
	{
		ToProtoData(input.protection_zones[i], output->add_protection_zones());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianLimitation input, Kinova::Api::Base::CartesianLimitation *output)
{
	
	output->set_type((Kinova::Api::Base::LimitationType)input.type);
	output->set_translation(input.translation);
	output->set_orientation(input.orientation);
	
	return 0;
}
int ToProtoData(kortex_driver::TwistLimitation input, Kinova::Api::Base::TwistLimitation *output)
{
	
	output->set_linear(input.linear);
	output->set_angular(input.angular);
	
	return 0;
}
int ToProtoData(kortex_driver::WrenchLimitation input, Kinova::Api::Base::WrenchLimitation *output)
{
	
	output->set_force(input.force);
	output->set_torque(input.torque);
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianLimitationList input, Kinova::Api::Base::CartesianLimitationList *output)
{
	 
	output->clear_limitations();
	for(int i = 0; i < input.limitations.size(); i++)
	{
		ToProtoData(input.limitations[i], output->add_limitations());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::JointLimitation input, Kinova::Api::Base::JointLimitation *output)
{
	
	output->set_joint_identifier(input.joint_identifier);
	output->set_type((Kinova::Api::Base::LimitationType)input.type);
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::JointsLimitationsList input, Kinova::Api::Base::JointsLimitationsList *output)
{
	 
	output->clear_joints_limitations();
	for(int i = 0; i < input.joints_limitations.size(); i++)
	{
		ToProtoData(input.joints_limitations[i], output->add_joints_limitations());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Query input, Kinova::Api::Base::Query *output)
{
	 
	ToProtoData(input.start_timestamp, output->mutable_start_timestamp()); 
	ToProtoData(input.end_timestamp, output->mutable_end_timestamp());
	output->set_username(input.username);
	
	return 0;
}
int ToProtoData(kortex_driver::ConfigurationChangeNotification input, Kinova::Api::Base::ConfigurationChangeNotification *output)
{
	
	output->set_event((Kinova::Api::Base::ConfigurationNotificationEvent)input.event); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	if(input.oneof_configuration_change.sequence_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.sequence_handle[0], output->mutable_sequence_handle());
	}
	if(input.oneof_configuration_change.action_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.action_handle[0], output->mutable_action_handle());
	}
	if(input.oneof_configuration_change.mapping_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.mapping_handle[0], output->mutable_mapping_handle());
	}
	if(input.oneof_configuration_change.map_group_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.map_group_handle[0], output->mutable_map_group_handle());
	}
	if(input.oneof_configuration_change.map_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.map_handle[0], output->mutable_map_handle());
	}
	if(input.oneof_configuration_change.user_profile_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.user_profile_handle[0], output->mutable_user_profile_handle());
	}
	if(input.oneof_configuration_change.protection_zone_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.protection_zone_handle[0], output->mutable_protection_zone_handle());
	}
	if(input.oneof_configuration_change.safety_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.safety_handle[0], output->mutable_safety_handle());
	}
	if(input.oneof_configuration_change.network_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.network_handle[0], output->mutable_network_handle());
	}
	if(input.oneof_configuration_change.ssid.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.ssid[0], output->mutable_ssid());
	}
	if(input.oneof_configuration_change.controller_handle.size() > 0)
	{
		ToProtoData(input.oneof_configuration_change.controller_handle[0], output->mutable_controller_handle());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MappingInfoNotification input, Kinova::Api::Base::MappingInfoNotification *output)
{
	
	output->set_controller_identifier(input.controller_identifier); 
	ToProtoData(input.active_map_handle, output->mutable_active_map_handle()); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection()); 
	ToProtoData(input.mapping_handle, output->mutable_mapping_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::Base_ControlModeInformation input, Kinova::Api::Base::ControlModeInformation *output)
{
	
	output->set_mode((Kinova::Api::Base::ControlMode)input.mode);
	
	return 0;
}
int ToProtoData(kortex_driver::Base_ControlModeNotification input, Kinova::Api::Base::ControlModeNotification *output)
{
	
	output->set_control_mode((Kinova::Api::Base::ControlMode)input.control_mode); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::ServoingModeInformation input, Kinova::Api::Base::ServoingModeInformation *output)
{
	
	output->set_servoing_mode((Kinova::Api::Base::ServoingMode)input.servoing_mode);
	
	return 0;
}
int ToProtoData(kortex_driver::OperatingModeInformation input, Kinova::Api::Base::OperatingModeInformation *output)
{
	
	output->set_operating_mode((Kinova::Api::Base::OperatingMode)input.operating_mode); 
	ToProtoData(input.device_handle, output->mutable_device_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::OperatingModeNotification input, Kinova::Api::Base::OperatingModeNotification *output)
{
	
	output->set_operating_mode((Kinova::Api::Base::OperatingMode)input.operating_mode); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection()); 
	ToProtoData(input.device_handle, output->mutable_device_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::ServoingModeNotification input, Kinova::Api::Base::ServoingModeNotification *output)
{
	
	output->set_servoing_mode((Kinova::Api::Base::ServoingMode)input.servoing_mode); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceInfoNotification input, Kinova::Api::Base::SequenceInfoNotification *output)
{
	
	output->set_event_identifier((Kinova::Api::Base::EventIdSequenceInfoNotification)input.event_identifier); 
	ToProtoData(input.sequence_handle, output->mutable_sequence_handle());
	output->set_task_index(input.task_index);
	output->set_group_identifier(input.group_identifier); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle());
	output->set_abort_details((Kinova::Api::SubErrorCodes)input.abort_details); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceInformation input, Kinova::Api::Base::SequenceInformation *output)
{
	
	output->set_event_identifier((Kinova::Api::Base::EventIdSequenceInfoNotification)input.event_identifier);
	output->set_task_index(input.task_index);
	output->set_task_identifier(input.task_identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneNotification input, Kinova::Api::Base::ProtectionZoneNotification *output)
{
	
	output->set_event((Kinova::Api::Base::ProtectionZoneEvent)input.event); 
	ToProtoData(input.handle, output->mutable_handle()); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneInformation input, Kinova::Api::Base::ProtectionZoneInformation *output)
{
	
	output->set_event((Kinova::Api::Base::ProtectionZoneEvent)input.event);
	
	return 0;
}
int ToProtoData(kortex_driver::UserNotification input, Kinova::Api::Base::UserNotification *output)
{
	
	output->set_user_event((Kinova::Api::Base::UserEvent)input.user_event); 
	ToProtoData(input.modified_user, output->mutable_modified_user()); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerHandle input, Kinova::Api::Base::ControllerHandle *output)
{
	
	output->set_type((Kinova::Api::Base::ControllerType)input.type);
	output->set_controller_identifier(input.controller_identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerElementHandle input, Kinova::Api::Base::ControllerElementHandle *output)
{
	 
	ToProtoData(input.controller_handle, output->mutable_controller_handle());
	if(input.oneof_identifier.button.size() > 0)
	{
		output->set_button(input.oneof_identifier.button[0]);
	}
	if(input.oneof_identifier.axis.size() > 0)
	{
		output->set_axis(input.oneof_identifier.axis[0]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerNotification input, Kinova::Api::Base::ControllerNotification *output)
{
	 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	if(input.oneof_state.controller_state.size() > 0)
	{
		ToProtoData(input.oneof_state.controller_state[0], output->mutable_controller_state());
	}
	if(input.oneof_state.controller_element.size() > 0)
	{
		ToProtoData(input.oneof_state.controller_element[0], output->mutable_controller_element());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerList input, Kinova::Api::Base::ControllerList *output)
{
	 
	output->clear_handles();
	for(int i = 0; i < input.handles.size(); i++)
	{
		ToProtoData(input.handles[i], output->add_handles());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerState input, Kinova::Api::Base::ControllerState *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_event_type((Kinova::Api::Base::ControllerEventType)input.event_type);
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerElementState input, Kinova::Api::Base::ControllerElementState *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_event_type((Kinova::Api::Base::ControllerElementEventType)input.event_type);
	output->set_axis_value(input.axis_value);
	
	return 0;
}
int ToProtoData(kortex_driver::ActionNotification input, Kinova::Api::Base::ActionNotification *output)
{
	
	output->set_action_event((Kinova::Api::Base::ActionEvent)input.action_event); 
	ToProtoData(input.handle, output->mutable_handle()); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle());
	output->set_abort_details((Kinova::Api::SubErrorCodes)input.abort_details); 
	ToProtoData(input.connection, output->mutable_connection()); 
	output->clear_trajectory_info();
	for(int i = 0; i < input.trajectory_info.size(); i++)
	{
		ToProtoData(input.trajectory_info[i], output->add_trajectory_info());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::TrajectoryInfo input, Kinova::Api::Base::TrajectoryInfo *output)
{
	
	output->set_trajectory_info_type((Kinova::Api::Base::TrajectoryInfoType)input.trajectory_info_type);
	output->set_waypoint_index(input.waypoint_index);
	output->set_joint_index(input.joint_index);
	
	return 0;
}
int ToProtoData(kortex_driver::ActionExecutionState input, Kinova::Api::Base::ActionExecutionState *output)
{
	
	output->set_action_event((Kinova::Api::Base::ActionEvent)input.action_event); 
	ToProtoData(input.handle, output->mutable_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::RobotEventNotification input, Kinova::Api::Base::RobotEventNotification *output)
{
	
	output->set_event((Kinova::Api::Base::RobotEvent)input.event); 
	ToProtoData(input.handle, output->mutable_handle()); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::FactoryNotification input, Kinova::Api::Base::FactoryNotification *output)
{
	
	output->set_event((Kinova::Api::Base::FactoryEvent)input.event); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::NetworkNotification input, Kinova::Api::Base::NetworkNotification *output)
{
	
	output->set_event((Kinova::Api::Base::NetworkEvent)input.event); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::ConfigurationChangeNotificationList input, Kinova::Api::Base::ConfigurationChangeNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MappingInfoNotificationList input, Kinova::Api::Base::MappingInfoNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ControlModeNotificationList input, Kinova::Api::Base::ControlModeNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::OperatingModeNotificationList input, Kinova::Api::Base::OperatingModeNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ServoingModeNotificationList input, Kinova::Api::Base::ServoingModeNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SequenceInfoNotificationList input, Kinova::Api::Base::SequenceInfoNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneNotificationList input, Kinova::Api::Base::ProtectionZoneNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::UserNotificationList input, Kinova::Api::Base::UserNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyNotificationList input, Kinova::Api::Base::SafetyNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerNotificationList input, Kinova::Api::Base::ControllerNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ActionNotificationList input, Kinova::Api::Base::ActionNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::RobotEventNotificationList input, Kinova::Api::Base::RobotEventNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::NetworkNotificationList input, Kinova::Api::Base::NetworkNotificationList *output)
{
	 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MappingHandle input, Kinova::Api::Base::MappingHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::SafetyEvent input, Kinova::Api::Base::SafetyEvent *output)
{
	 
	ToProtoData(input.safety_handle, output->mutable_safety_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerEvent input, Kinova::Api::Base::ControllerEvent *output)
{
	
	output->set_input_type((Kinova::Api::Base::ControllerInputType)input.input_type);
	output->set_behavior((Kinova::Api::Base::ControllerBehavior)input.behavior);
	output->set_input_identifier(input.input_identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::GpioEvent input, Kinova::Api::Base::GpioEvent *output)
{
	
	output->set_input_type((Kinova::Api::Base::ControllerInputType)input.input_type);
	output->set_behavior((Kinova::Api::Base::GpioBehavior)input.behavior);
	output->set_input_identifier(input.input_identifier);
	
	return 0;
}
int ToProtoData(kortex_driver::MapEvent input, Kinova::Api::Base::MapEvent *output)
{
	
	output->set_name(input.name);
	if(input.oneof_events.safety_event.size() > 0)
	{
		ToProtoData(input.oneof_events.safety_event[0], output->mutable_safety_event());
	}
	if(input.oneof_events.gpio_event.size() > 0)
	{
		ToProtoData(input.oneof_events.gpio_event[0], output->mutable_gpio_event());
	}
	if(input.oneof_events.controller_event.size() > 0)
	{
		ToProtoData(input.oneof_events.controller_event[0], output->mutable_controller_event());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MapElement input, Kinova::Api::Base::MapElement *output)
{
	 
	ToProtoData(input.event, output->mutable_event()); 
	ToProtoData(input.action, output->mutable_action());
	output->set_name(input.name);
	
	return 0;
}
int ToProtoData(kortex_driver::ActivateMapHandle input, Kinova::Api::Base::ActivateMapHandle *output)
{
	 
	ToProtoData(input.mapping_handle, output->mutable_mapping_handle()); 
	ToProtoData(input.map_group_handle, output->mutable_map_group_handle()); 
	ToProtoData(input.map_handle, output->mutable_map_handle());
	
	return 0;
}
int ToProtoData(kortex_driver::Map input, Kinova::Api::Base::Map *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name); 
	output->clear_elements();
	for(int i = 0; i < input.elements.size(); i++)
	{
		ToProtoData(input.elements[i], output->add_elements());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MapHandle input, Kinova::Api::Base::MapHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::MapList input, Kinova::Api::Base::MapList *output)
{
	 
	output->clear_map_list();
	for(int i = 0; i < input.map_list.size(); i++)
	{
		ToProtoData(input.map_list[i], output->add_map_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::MapGroupHandle input, Kinova::Api::Base::MapGroupHandle *output)
{
	
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);
	
	return 0;
}
int ToProtoData(kortex_driver::MapGroup input, Kinova::Api::Base::MapGroup *output)
{
	 
	ToProtoData(input.group_handle, output->mutable_group_handle());
	output->set_name(input.name); 
	ToProtoData(input.related_mapping_handle, output->mutable_related_mapping_handle()); 
	ToProtoData(input.parent_group_handle, output->mutable_parent_group_handle()); 
	output->clear_children_map_group_handles();
	for(int i = 0; i < input.children_map_group_handles.size(); i++)
	{
		ToProtoData(input.children_map_group_handles[i], output->add_children_map_group_handles());
	} 
	output->clear_map_handles();
	for(int i = 0; i < input.map_handles.size(); i++)
	{
		ToProtoData(input.map_handles[i], output->add_map_handles());
	}
	output->set_application_data(input.application_data);
	
	return 0;
}
int ToProtoData(kortex_driver::MapGroupList input, Kinova::Api::Base::MapGroupList *output)
{
	 
	output->clear_map_groups();
	for(int i = 0; i < input.map_groups.size(); i++)
	{
		ToProtoData(input.map_groups[i], output->add_map_groups());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Mapping input, Kinova::Api::Base::Mapping *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name);
	output->set_controller_identifier(input.controller_identifier); 
	ToProtoData(input.active_map_group_handle, output->mutable_active_map_group_handle()); 
	output->clear_map_group_handles();
	for(int i = 0; i < input.map_group_handles.size(); i++)
	{
		ToProtoData(input.map_group_handles[i], output->add_map_group_handles());
	} 
	ToProtoData(input.active_map_handle, output->mutable_active_map_handle()); 
	output->clear_map_handles();
	for(int i = 0; i < input.map_handles.size(); i++)
	{
		ToProtoData(input.map_handles[i], output->add_map_handles());
	}
	output->set_application_data(input.application_data);
	
	return 0;
}
int ToProtoData(kortex_driver::MappingList input, Kinova::Api::Base::MappingList *output)
{
	 
	output->clear_mappings();
	for(int i = 0; i < input.mappings.size(); i++)
	{
		ToProtoData(input.mappings[i], output->add_mappings());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::TransformationMatrix input, Kinova::Api::Base::TransformationMatrix *output)
{
	 
	ToProtoData(input.r0, output->mutable_r0()); 
	ToProtoData(input.r1, output->mutable_r1()); 
	ToProtoData(input.r2, output->mutable_r2()); 
	ToProtoData(input.r3, output->mutable_r3());
	
	return 0;
}
int ToProtoData(kortex_driver::TransformationRow input, Kinova::Api::Base::TransformationRow *output)
{
	
	output->set_c0(input.c0);
	output->set_c1(input.c1);
	output->set_c2(input.c2);
	output->set_c3(input.c3);
	
	return 0;
}
int ToProtoData(kortex_driver::Pose input, Kinova::Api::Base::Pose *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	output->set_theta_x(input.theta_x);
	output->set_theta_y(input.theta_y);
	output->set_theta_z(input.theta_z);
	
	return 0;
}
int ToProtoData(kortex_driver::Base_Position input, Kinova::Api::Base::Position *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	
	return 0;
}
int ToProtoData(kortex_driver::Orientation input, Kinova::Api::Base::Orientation *output)
{
	
	output->set_theta_x(input.theta_x);
	output->set_theta_y(input.theta_y);
	output->set_theta_z(input.theta_z);
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianSpeed input, Kinova::Api::Base::CartesianSpeed *output)
{
	
	output->set_translation(input.translation);
	output->set_orientation(input.orientation);
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianTrajectoryConstraint input, Kinova::Api::Base::CartesianTrajectoryConstraint *output)
{
	
	if(input.oneof_type.speed.size() > 0)
	{
		ToProtoData(input.oneof_type.speed[0], output->mutable_speed());
	}
	if(input.oneof_type.duration.size() > 0)
	{
		output->set_duration(input.oneof_type.duration[0]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::JointTrajectoryConstraint input, Kinova::Api::Base::JointTrajectoryConstraint *output)
{
	
	output->set_type((Kinova::Api::Base::JointTrajectoryConstraintType)input.type);
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::Wrench input, Kinova::Api::Base::Wrench *output)
{
	
	output->set_force_x(input.force_x);
	output->set_force_y(input.force_y);
	output->set_force_z(input.force_z);
	output->set_torque_x(input.torque_x);
	output->set_torque_y(input.torque_y);
	output->set_torque_z(input.torque_z);
	
	return 0;
}
int ToProtoData(kortex_driver::Twist input, Kinova::Api::Base::Twist *output)
{
	
	output->set_linear_x(input.linear_x);
	output->set_linear_y(input.linear_y);
	output->set_linear_z(input.linear_z);
	output->set_angular_x(input.angular_x);
	output->set_angular_y(input.angular_y);
	output->set_angular_z(input.angular_z);
	
	return 0;
}
int ToProtoData(kortex_driver::Admittance input, Kinova::Api::Base::Admittance *output)
{
	
	output->set_admittance_mode((Kinova::Api::Base::AdmittanceMode)input.admittance_mode);
	
	return 0;
}
int ToProtoData(kortex_driver::ConstrainedPose input, Kinova::Api::Base::ConstrainedPose *output)
{
	 
	ToProtoData(input.target_pose, output->mutable_target_pose()); 
	ToProtoData(input.constraint, output->mutable_constraint());
	
	return 0;
}
int ToProtoData(kortex_driver::ConstrainedPosition input, Kinova::Api::Base::ConstrainedPosition *output)
{
	 
	ToProtoData(input.target_position, output->mutable_target_position()); 
	ToProtoData(input.constraint, output->mutable_constraint());
	
	return 0;
}
int ToProtoData(kortex_driver::ConstrainedOrientation input, Kinova::Api::Base::ConstrainedOrientation *output)
{
	 
	ToProtoData(input.target_orientation, output->mutable_target_orientation()); 
	ToProtoData(input.constraint, output->mutable_constraint());
	
	return 0;
}
int ToProtoData(kortex_driver::WrenchCommand input, Kinova::Api::Base::WrenchCommand *output)
{
	
	output->set_reference_frame((Kinova::Api::Common::CartesianReferenceFrame)input.reference_frame);
	output->set_mode((Kinova::Api::Base::WrenchMode)input.mode); 
	ToProtoData(input.wrench, output->mutable_wrench());
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::TwistCommand input, Kinova::Api::Base::TwistCommand *output)
{
	
	output->set_reference_frame((Kinova::Api::Common::CartesianReferenceFrame)input.reference_frame); 
	ToProtoData(input.twist, output->mutable_twist());
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::ConstrainedJointAngles input, Kinova::Api::Base::ConstrainedJointAngles *output)
{
	 
	ToProtoData(input.joint_angles, output->mutable_joint_angles()); 
	ToProtoData(input.constraint, output->mutable_constraint());
	
	return 0;
}
int ToProtoData(kortex_driver::ConstrainedJointAngle input, Kinova::Api::Base::ConstrainedJointAngle *output)
{
	
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value); 
	ToProtoData(input.constraint, output->mutable_constraint());
	
	return 0;
}
int ToProtoData(kortex_driver::JointAngles input, Kinova::Api::Base::JointAngles *output)
{
	 
	output->clear_joint_angles();
	for(int i = 0; i < input.joint_angles.size(); i++)
	{
		ToProtoData(input.joint_angles[i], output->add_joint_angles());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::JointAngle input, Kinova::Api::Base::JointAngle *output)
{
	
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::Base_JointSpeeds input, Kinova::Api::Base::JointSpeeds *output)
{
	 
	output->clear_joint_speeds();
	for(int i = 0; i < input.joint_speeds.size(); i++)
	{
		ToProtoData(input.joint_speeds[i], output->add_joint_speeds());
	}
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::JointSpeed input, Kinova::Api::Base::JointSpeed *output)
{
	
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value);
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::JointTorques input, Kinova::Api::Base::JointTorques *output)
{
	 
	output->clear_joint_torques();
	for(int i = 0; i < input.joint_torques.size(); i++)
	{
		ToProtoData(input.joint_torques[i], output->add_joint_torques());
	}
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::JointTorque input, Kinova::Api::Base::JointTorque *output)
{
	
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value);
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::GripperCommand input, Kinova::Api::Base::GripperCommand *output)
{
	
	output->set_mode((Kinova::Api::Base::GripperMode)input.mode); 
	ToProtoData(input.gripper, output->mutable_gripper());
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::GripperRequest input, Kinova::Api::Base::GripperRequest *output)
{
	
	output->set_mode((Kinova::Api::Base::GripperMode)input.mode);
	
	return 0;
}
int ToProtoData(kortex_driver::Gripper input, Kinova::Api::Base::Gripper *output)
{
	 
	output->clear_finger();
	for(int i = 0; i < input.finger.size(); i++)
	{
		ToProtoData(input.finger[i], output->add_finger());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::Finger input, Kinova::Api::Base::Finger *output)
{
	
	output->set_finger_identifier(input.finger_identifier);
	output->set_value(input.value);
	
	return 0;
}
int ToProtoData(kortex_driver::GpioCommand input, Kinova::Api::Base::GpioCommand *output)
{
	
	output->set_port_identifier(input.port_identifier);
	output->set_pin_identifier(input.pin_identifier);
	output->set_action((Kinova::Api::Base::GpioAction)input.action);
	output->set_period(input.period);
	
	return 0;
}
int ToProtoData(kortex_driver::SystemTime input, Kinova::Api::Base::SystemTime *output)
{
	
	output->set_sec(input.sec);
	output->set_min(input.min);
	output->set_hour(input.hour);
	output->set_mday(input.mday);
	output->set_mon(input.mon);
	output->set_year(input.year);
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerConfigurationMode input, Kinova::Api::Base::ControllerConfigurationMode *output)
{
	
	output->set_enable(input.enable);
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerConfiguration input, Kinova::Api::Base::ControllerConfiguration *output)
{
	 
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name); 
	ToProtoData(input.active_mapping_handle, output->mutable_active_mapping_handle());
	output->set_analog_input_identifier_enum(input.analog_input_identifier_enum);
	output->set_digital_input_identifier_enum(input.digital_input_identifier_enum);
	
	return 0;
}
int ToProtoData(kortex_driver::ControllerConfigurationList input, Kinova::Api::Base::ControllerConfigurationList *output)
{
	 
	output->clear_controller_configurations();
	for(int i = 0; i < input.controller_configurations.size(); i++)
	{
		ToProtoData(input.controller_configurations[i], output->add_controller_configurations());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ActuatorInformation input, Kinova::Api::Base::ActuatorInformation *output)
{
	
	output->set_count(input.count);
	
	return 0;
}
int ToProtoData(kortex_driver::ArmStateInformation input, Kinova::Api::Base::ArmStateInformation *output)
{
	
	output->set_active_state((Kinova::Api::Common::ArmState)input.active_state); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::ArmStateNotification input, Kinova::Api::Base::ArmStateNotification *output)
{
	
	output->set_active_state((Kinova::Api::Common::ArmState)input.active_state); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::Base_CapSenseConfig input, Kinova::Api::Base::CapSenseConfig *output)
{
	
	output->set_identifier(input.identifier);
	output->set_mode((Kinova::Api::Base::CapSenseMode)input.mode);
	output->set_threshold_a(input.threshold_a);
	output->set_threshold_b(input.threshold_b);
	output->set_sensitivity_a(input.sensitivity_a);
	output->set_sensitivity_b(input.sensitivity_b);
	
	return 0;
}
int ToProtoData(kortex_driver::BridgeList input, Kinova::Api::Base::BridgeList *output)
{
	 
	output->clear_bridgeconfig();
	for(int i = 0; i < input.bridgeConfig.size(); i++)
	{
		ToProtoData(input.bridgeConfig[i], output->add_bridgeconfig());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::BridgeResult input, Kinova::Api::Base::BridgeResult *output)
{
	 
	ToProtoData(input.bridge_id, output->mutable_bridge_id());
	output->set_status((Kinova::Api::Base::BridgeStatus)input.status);
	
	return 0;
}
int ToProtoData(kortex_driver::BridgeIdentifier input, Kinova::Api::Base::BridgeIdentifier *output)
{
	
	output->set_bridge_id(input.bridge_id);
	
	return 0;
}
int ToProtoData(kortex_driver::BridgeConfig input, Kinova::Api::Base::BridgeConfig *output)
{
	
	output->set_device_identifier(input.device_identifier);
	output->set_bridgetype((Kinova::Api::Base::BridgeType)input.bridgetype); 
	ToProtoData(input.port_config, output->mutable_port_config()); 
	ToProtoData(input.bridge_id, output->mutable_bridge_id());
	
	return 0;
}
int ToProtoData(kortex_driver::BridgePortConfig input, Kinova::Api::Base::BridgePortConfig *output)
{
	
	output->set_target_port(input.target_port);
	output->set_out_port(input.out_port);
	
	return 0;
}
int ToProtoData(kortex_driver::PreComputedJointTrajectory input, Kinova::Api::Base::PreComputedJointTrajectory *output)
{
	
	output->set_mode((Kinova::Api::Base::TrajectoryContinuityMode)input.mode); 
	output->clear_trajectory_elements();
	for(int i = 0; i < input.trajectory_elements.size(); i++)
	{
		ToProtoData(input.trajectory_elements[i], output->add_trajectory_elements());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::PreComputedJointTrajectoryElement input, Kinova::Api::Base::PreComputedJointTrajectoryElement *output)
{
	
	output->clear_joint_angles();
	for(int i = 0; i < input.joint_angles.size(); i++)
	{
		output->add_joint_angles(input.joint_angles[i]);
	}
	output->clear_joint_speeds();
	for(int i = 0; i < input.joint_speeds.size(); i++)
	{
		output->add_joint_speeds(input.joint_speeds[i]);
	}
	output->clear_joint_accelerations();
	for(int i = 0; i < input.joint_accelerations.size(); i++)
	{
		output->add_joint_accelerations(input.joint_accelerations[i]);
	}
	output->set_time_from_start(input.time_from_start);
	
	return 0;
}
int ToProtoData(kortex_driver::TrajectoryErrorElement input, Kinova::Api::Base::TrajectoryErrorElement *output)
{
	
	output->set_error_type((Kinova::Api::Base::TrajectoryErrorType)input.error_type);
	output->set_error_identifier((Kinova::Api::Base::TrajectoryErrorIdentifier)input.error_identifier);
	output->set_error_value(input.error_value);
	output->set_min_value(input.min_value);
	output->set_max_value(input.max_value);
	output->set_index(input.index);
	output->set_message(input.message);
	output->set_waypoint_index(input.waypoint_index);
	
	return 0;
}
int ToProtoData(kortex_driver::TrajectoryErrorReport input, Kinova::Api::Base::TrajectoryErrorReport *output)
{
	 
	output->clear_trajectory_error_elements();
	for(int i = 0; i < input.trajectory_error_elements.size(); i++)
	{
		ToProtoData(input.trajectory_error_elements[i], output->add_trajectory_error_elements());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::WaypointValidationReport input, Kinova::Api::Base::WaypointValidationReport *output)
{
	 
	ToProtoData(input.trajectory_error_report, output->mutable_trajectory_error_report()); 
	ToProtoData(input.optimal_waypoint_list, output->mutable_optimal_waypoint_list());
	
	return 0;
}
int ToProtoData(kortex_driver::Waypoint input, Kinova::Api::Base::Waypoint *output)
{
	
	output->set_name(input.name);
	if(input.oneof_type_of_waypoint.angular_waypoint.size() > 0)
	{
		ToProtoData(input.oneof_type_of_waypoint.angular_waypoint[0], output->mutable_angular_waypoint());
	}
	if(input.oneof_type_of_waypoint.cartesian_waypoint.size() > 0)
	{
		ToProtoData(input.oneof_type_of_waypoint.cartesian_waypoint[0], output->mutable_cartesian_waypoint());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::AngularWaypoint input, Kinova::Api::Base::AngularWaypoint *output)
{
	
	output->clear_angles();
	for(int i = 0; i < input.angles.size(); i++)
	{
		output->add_angles(input.angles[i]);
	}
	output->clear_maximum_velocities();
	for(int i = 0; i < input.maximum_velocities.size(); i++)
	{
		output->add_maximum_velocities(input.maximum_velocities[i]);
	}
	output->set_duration(input.duration);
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianWaypoint input, Kinova::Api::Base::CartesianWaypoint *output)
{
	 
	ToProtoData(input.pose, output->mutable_pose());
	output->set_reference_frame((Kinova::Api::Common::CartesianReferenceFrame)input.reference_frame);
	output->set_maximum_linear_velocity(input.maximum_linear_velocity);
	output->set_maximum_angular_velocity(input.maximum_angular_velocity);
	output->set_blending_radius(input.blending_radius);
	
	return 0;
}
int ToProtoData(kortex_driver::WaypointList input, Kinova::Api::Base::WaypointList *output)
{
	 
	output->clear_waypoints();
	for(int i = 0; i < input.waypoints.size(); i++)
	{
		ToProtoData(input.waypoints[i], output->add_waypoints());
	}
	output->set_duration(input.duration);
	output->set_use_optimal_blending(input.use_optimal_blending);
	
	return 0;
}
int ToProtoData(kortex_driver::KinematicTrajectoryConstraints input, Kinova::Api::Base::KinematicTrajectoryConstraints *output)
{
	
	output->clear_angular_velocities();
	for(int i = 0; i < input.angular_velocities.size(); i++)
	{
		output->add_angular_velocities(input.angular_velocities[i]);
	}
	output->set_linear_velocity(input.linear_velocity);
	output->set_angular_velocity(input.angular_velocity);
	
	return 0;
}
int ToProtoData(kortex_driver::FirmwareBundleVersions input, Kinova::Api::Base::FirmwareBundleVersions *output)
{
	
	output->set_main_bundle_version(input.main_bundle_version); 
	output->clear_components_versions();
	for(int i = 0; i < input.components_versions.size(); i++)
	{
		ToProtoData(input.components_versions[i], output->add_components_versions());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::FirmwareComponentVersion input, Kinova::Api::Base::FirmwareComponentVersion *output)
{
	
	output->set_name(input.name);
	output->set_version(input.version);
	output->set_device_id(input.device_id);
	
	return 0;
}
int ToProtoData(kortex_driver::IKData input, Kinova::Api::Base::IKData *output)
{
	 
	ToProtoData(input.cartesian_pose, output->mutable_cartesian_pose()); 
	ToProtoData(input.guess, output->mutable_guess());
	
	return 0;
}
