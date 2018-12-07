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
 
#include "base_proto_converter.h"

#include "common_proto_converter.h"


int ToProtoData(kortex_driver::FullUserProfile input, FullUserProfile *output)
{
	ToProtoData(input.user_profile, output->mutable_user_profile());
	output->set_password(input.password);

	return 0;
}
int ToProtoData(kortex_driver::UserProfile input, UserProfile *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_username(input.username);
	output->set_firstname(input.firstname);
	output->set_lastname(input.lastname);
	output->set_application_data(input.application_data);

	return 0;
}
int ToProtoData(kortex_driver::UserProfileList input, UserProfileList *output)
{ 
	output->clear_user_profiles();
	for(int i = 0; i < input.user_profiles.size(); i++)
	{
		ToProtoData(input.user_profiles[i], output->add_user_profiles());
	}

	return 0;
}
int ToProtoData(kortex_driver::UserList input, UserList *output)
{ 
	output->clear_user_handles();
	for(int i = 0; i < input.user_handles.size(); i++)
	{
		ToProtoData(input.user_handles[i], output->add_user_handles());
	}

	return 0;
}
int ToProtoData(kortex_driver::PasswordChange input, PasswordChange *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_old_password(input.old_password);
	output->set_new_password(input.new_password);

	return 0;
}
int ToProtoData(kortex_driver::SequenceHandle input, SequenceHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_driver::AdvancedSequenceHandle input, AdvancedSequenceHandle *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_in_loop(input.in_loop);

	return 0;
}
int ToProtoData(kortex_driver::SequenceTaskHandle input, SequenceTaskHandle *output)
{
	ToProtoData(input.sequence_handle, output->mutable_sequence_handle());
	output->set_task_index(input.task_index);

	return 0;
}
int ToProtoData(kortex_driver::SequenceTask input, SequenceTask *output)
{
	output->set_group_identifier(input.group_identifier);
	ToProtoData(input.action, output->mutable_action());
	output->set_application_data(input.application_data);

	return 0;
}
int ToProtoData(kortex_driver::Sequence input, Sequence *output)
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
int ToProtoData(kortex_driver::SequenceList input, SequenceList *output)
{ 
	output->clear_sequence_list();
	for(int i = 0; i < input.sequence_list.size(); i++)
	{
		ToProtoData(input.sequence_list[i], output->add_sequence_list());
	}

	return 0;
}
int ToProtoData(kortex_driver::AppendActionInformation input, AppendActionInformation *output)
{
	ToProtoData(input.sequence_handle, output->mutable_sequence_handle());
	ToProtoData(input.action, output->mutable_action());

	return 0;
}
int ToProtoData(kortex_driver::ActionHandle input, ActionHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_action_type((Kinova::Api::Base::ActionType)input.action_type);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_driver::RequestedActionType input, RequestedActionType *output)
{
	output->set_action_type((Kinova::Api::Base::ActionType)input.action_type);

	return 0;
}
int ToProtoData(kortex_driver::Action input, Action *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_name(input.name);
	output->set_application_data(input.application_data);
	
	if(input.oneof_action_parameters.send_twist_command.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.send_twist_command[0], output->mutable_send_twist_command());
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
		output->set_toggle_admittance_mode((AdmittanceMode)input.oneof_action_parameters.toggle_admittance_mode[0]);
	}
	if(input.oneof_action_parameters.switch_control_mapping.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.switch_control_mapping[0], output->mutable_switch_control_mapping());
	}
	if(input.oneof_action_parameters.navigate_joints.size() > 0)
	{
		output->set_navigate_joints((JointNavigationDirection)input.oneof_action_parameters.navigate_joints[0]);
	}
	if(input.oneof_action_parameters.navigate_mappings.size() > 0)
	{
		output->set_navigate_mappings((NavigationDirection)input.oneof_action_parameters.navigate_mappings[0]);
	}
	if(input.oneof_action_parameters.change_twist.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.change_twist[0], output->mutable_change_twist());
	}
	if(input.oneof_action_parameters.change_joint_speeds.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.change_joint_speeds[0], output->mutable_change_joint_speeds());
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
	if(input.oneof_action_parameters.stop_action.size() > 0)
	{
		ToProtoData(input.oneof_action_parameters.stop_action[0], output->mutable_stop_action());
	}
	

	return 0;
}
int ToProtoData(kortex_driver::SwitchControlMapping input, SwitchControlMapping *output)
{
	output->set_controller_identifier(input.controller_identifier);
	ToProtoData(input.map_group_handle, output->mutable_map_group_handle());
	ToProtoData(input.map_handle, output->mutable_map_handle());

	return 0;
}
int ToProtoData(kortex_driver::ChangeTwist input, ChangeTwist *output)
{
	output->set_linear(input.linear);
	output->set_angular(input.angular);

	return 0;
}
int ToProtoData(kortex_driver::ChangeJointSpeeds input, ChangeJointSpeeds *output)
{
	ToProtoData(input.joint_speeds, output->mutable_joint_speeds());

	return 0;
}
int ToProtoData(kortex_driver::EmergencyStop input, EmergencyStop *output)
{

	return 0;
}
int ToProtoData(kortex_driver::Faults input, Faults *output)
{

	return 0;
}
int ToProtoData(kortex_driver::Delay input, Delay *output)
{
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_driver::Stop input, Stop *output)
{

	return 0;
}
int ToProtoData(kortex_driver::ActionList input, ActionList *output)
{ 
	output->clear_action_list();
	for(int i = 0; i < input.action_list.size(); i++)
	{
		ToProtoData(input.action_list[i], output->add_action_list());
	}

	return 0;
}
int ToProtoData(kortex_driver::Timeout input, Timeout *output)
{
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_driver::Ssid input, Ssid *output)
{
	output->set_identifier(input.identifier);

	return 0;
}
int ToProtoData(kortex_driver::CommunicationInterfaceConfiguration input, CommunicationInterfaceConfiguration *output)
{
	output->set_type((Kinova::Api::Base::NetworkType)input.type);
	output->set_enable(input.enable);

	return 0;
}
int ToProtoData(kortex_driver::NetworkHandle input, NetworkHandle *output)
{
	output->set_type((Kinova::Api::Base::NetworkType)input.type);

	return 0;
}
int ToProtoData(kortex_driver::IPv4Configuration input, IPv4Configuration *output)
{
	output->set_ip_address(input.ip_address);
	output->set_subnet_mask(input.subnet_mask);
	output->set_default_gateway(input.default_gateway);
	output->set_dhcp_enabled(input.dhcp_enabled);

	return 0;
}
int ToProtoData(kortex_driver::IPv4Information input, IPv4Information *output)
{
	output->set_ip_address(input.ip_address);
	output->set_subnet_mask(input.subnet_mask);
	output->set_default_gateway(input.default_gateway);

	return 0;
}
int ToProtoData(kortex_driver::FullIPv4Configuration input, FullIPv4Configuration *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	ToProtoData(input.ipv4_configuration, output->mutable_ipv4_configuration());

	return 0;
}
int ToProtoData(kortex_driver::WifiInformation input, WifiInformation *output)
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
int ToProtoData(kortex_driver::WifiInformationList input, WifiInformationList *output)
{ 
	output->clear_wifi_information_list();
	for(int i = 0; i < input.wifi_information_list.size(); i++)
	{
		ToProtoData(input.wifi_information_list[i], output->add_wifi_information_list());
	}

	return 0;
}
int ToProtoData(kortex_driver::WifiConfiguration input, WifiConfiguration *output)
{
	ToProtoData(input.ssid, output->mutable_ssid());
	output->set_security_key(input.security_key);
	output->set_connect_automatically(input.connect_automatically);

	return 0;
}
int ToProtoData(kortex_driver::WifiConfigurationList input, WifiConfigurationList *output)
{ 
	output->clear_wifi_configuration_list();
	for(int i = 0; i < input.wifi_configuration_list.size(); i++)
	{
		ToProtoData(input.wifi_configuration_list[i], output->add_wifi_configuration_list());
	}

	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneHandle input, ProtectionZoneHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_driver::RotationMatrixRow input, RotationMatrixRow *output)
{
	output->set_column1(input.column1);
	output->set_column2(input.column2);
	output->set_column3(input.column3);

	return 0;
}
int ToProtoData(kortex_driver::RotationMatrix input, RotationMatrix *output)
{
	ToProtoData(input.row1, output->mutable_row1());
	ToProtoData(input.row2, output->mutable_row2());
	ToProtoData(input.row3, output->mutable_row3());

	return 0;
}
int ToProtoData(kortex_driver::Point input, Point *output)
{
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);

	return 0;
}
int ToProtoData(kortex_driver::ZoneShape input, ZoneShape *output)
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
int ToProtoData(kortex_driver::ProtectionZone input, ProtectionZone *output)
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
int ToProtoData(kortex_driver::ProtectionZoneList input, ProtectionZoneList *output)
{ 
	output->clear_protection_zones();
	for(int i = 0; i < input.protection_zones.size(); i++)
	{
		ToProtoData(input.protection_zones[i], output->add_protection_zones());
	}

	return 0;
}
int ToProtoData(kortex_driver::LimitationTypeIdentifier input, LimitationTypeIdentifier *output)
{
	output->set_type((Kinova::Api::Base::LimitationType)input.type);

	return 0;
}
int ToProtoData(kortex_driver::CartesianLimitation input, CartesianLimitation *output)
{
	output->set_type((Kinova::Api::Base::LimitationType)input.type);
	output->set_translation(input.translation);
	output->set_orientation(input.orientation);

	return 0;
}
int ToProtoData(kortex_driver::CartesianLimitationList input, CartesianLimitationList *output)
{ 
	output->clear_limitations();
	for(int i = 0; i < input.limitations.size(); i++)
	{
		ToProtoData(input.limitations[i], output->add_limitations());
	}

	return 0;
}
int ToProtoData(kortex_driver::JointLimitationValue input, JointLimitationValue *output)
{
	output->set_type((Kinova::Api::Base::LimitationType)input.type);
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_driver::JointLimitationValueList input, JointLimitationValueList *output)
{ 
	output->clear_joint_limitation_values();
	for(int i = 0; i < input.joint_limitation_values.size(); i++)
	{
		ToProtoData(input.joint_limitation_values[i], output->add_joint_limitation_values());
	}

	return 0;
}
int ToProtoData(kortex_driver::JointLimitation input, JointLimitation *output)
{
	output->set_device_identifier(input.device_identifier);
	ToProtoData(input.limitation_value, output->mutable_limitation_value());

	return 0;
}
int ToProtoData(kortex_driver::JointLimitationTypeIdentifier input, JointLimitationTypeIdentifier *output)
{
	output->set_device_identifier(input.device_identifier);
	output->set_type((Kinova::Api::Base::LimitationType)input.type);

	return 0;
}
int ToProtoData(kortex_driver::Query input, Query *output)
{
	ToProtoData(input.start_timestamp, output->mutable_start_timestamp());
	ToProtoData(input.end_timestamp, output->mutable_end_timestamp());
	output->set_username(input.username);

	return 0;
}
int ToProtoData(kortex_driver::ConfigurationChangeNotification input, ConfigurationChangeNotification *output)
{
	output->set_event((Kinova::Api::Base::ConfigurationNotificationEvent)input.event);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::MappingInfoNotification input, MappingInfoNotification *output)
{
	output->set_controller_identifier(input.controller_identifier);
	ToProtoData(input.active_map_handle, output->mutable_active_map_handle());
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ControlModeInformation input, ControlModeInformation *output)
{
	output->set_mode((Kinova::Api::Base::ControlMode)input.mode);

	return 0;
}
int ToProtoData(kortex_driver::ControlModeNotification input, ControlModeNotification *output)
{
	output->set_control_mode((Kinova::Api::Base::ControlMode)input.control_mode);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ServoingModeInformation input, ServoingModeInformation *output)
{
	output->set_servoing_mode((Kinova::Api::Base::ServoingMode)input.servoing_mode);

	return 0;
}
int ToProtoData(kortex_driver::OperatingModeInformation input, OperatingModeInformation *output)
{
	output->set_operating_mode((Kinova::Api::Base::OperatingMode)input.operating_mode);
	ToProtoData(input.device_handle, output->mutable_device_handle());

	return 0;
}
int ToProtoData(kortex_driver::OperatingModeNotification input, OperatingModeNotification *output)
{
	output->set_operating_mode((Kinova::Api::Base::OperatingMode)input.operating_mode);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());
	ToProtoData(input.device_handle, output->mutable_device_handle());

	return 0;
}
int ToProtoData(kortex_driver::ServoingModeNotification input, ServoingModeNotification *output)
{
	output->set_servoing_mode((Kinova::Api::Base::ServoingMode)input.servoing_mode);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::SequenceInfoNotification input, SequenceInfoNotification *output)
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
int ToProtoData(kortex_driver::SequenceInformation input, SequenceInformation *output)
{
	output->set_event_identifier((Kinova::Api::Base::EventIdSequenceInfoNotification)input.event_identifier);
	output->set_task_index(input.task_index);
	output->set_task_identifier(input.task_identifier);

	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneNotification input, ProtectionZoneNotification *output)
{
	output->set_event((Kinova::Api::Base::ProtectionZoneEvent)input.event);
	ToProtoData(input.handle, output->mutable_handle());
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneInformation input, ProtectionZoneInformation *output)
{
	output->set_event((Kinova::Api::Base::ProtectionZoneEvent)input.event);

	return 0;
}
int ToProtoData(kortex_driver::UserNotification input, UserNotification *output)
{
	output->set_user_event((Kinova::Api::Base::UserEvent)input.user_event);
	ToProtoData(input.modified_user, output->mutable_modified_user());
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ControllerHandle input, ControllerHandle *output)
{
	output->set_type((Kinova::Api::Base::ControllerType)input.type);
	output->set_controller_identifier(input.controller_identifier);

	return 0;
}
int ToProtoData(kortex_driver::ControllerElementHandle input, ControllerElementHandle *output)
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
int ToProtoData(kortex_driver::ControllerNotification input, ControllerNotification *output)
{
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ControllerList input, ControllerList *output)
{ 
	output->clear_handles();
	for(int i = 0; i < input.handles.size(); i++)
	{
		ToProtoData(input.handles[i], output->add_handles());
	}

	return 0;
}
int ToProtoData(kortex_driver::ControllerState input, ControllerState *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_event_type((Kinova::Api::Base::ControllerEventType)input.event_type);

	return 0;
}
int ToProtoData(kortex_driver::ControllerElementState input, ControllerElementState *output)
{
	ToProtoData(input.handle, output->mutable_handle());
	output->set_event_type((Kinova::Api::Base::ControllerElementEventType)input.event_type);
	output->set_axis_value(input.axis_value);

	return 0;
}
int ToProtoData(kortex_driver::ActionNotification input, ActionNotification *output)
{
	output->set_action_event((Kinova::Api::Base::ActionEvent)input.action_event);
	ToProtoData(input.handle, output->mutable_handle());
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	output->set_abort_details((Kinova::Api::SubErrorCodes)input.abort_details);
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ActionExecutionState input, ActionExecutionState *output)
{
	output->set_action_event((Kinova::Api::Base::ActionEvent)input.action_event);
	ToProtoData(input.handle, output->mutable_handle());

	return 0;
}
int ToProtoData(kortex_driver::RobotEventNotification input, RobotEventNotification *output)
{
	output->set_event((Kinova::Api::Base::RobotEvent)input.event);
	ToProtoData(input.handle, output->mutable_handle());
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::FactoryNotification input, FactoryNotification *output)
{
	output->set_event((Kinova::Api::Base::FactoryEvent)input.event);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::NetworkNotification input, NetworkNotification *output)
{
	output->set_event((Kinova::Api::Base::NetworkEvent)input.event);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.user_handle, output->mutable_user_handle());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ConfigurationChangeNotificationList input, ConfigurationChangeNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::MappingInfoNotificationList input, MappingInfoNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::ControlModeNotificationList input, ControlModeNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::OperatingModeNotificationList input, OperatingModeNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::ServoingModeNotificationList input, ServoingModeNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::SequenceInfoNotificationList input, SequenceInfoNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::ProtectionZoneNotificationList input, ProtectionZoneNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::UserNotificationList input, UserNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::SafetyNotificationList input, SafetyNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::ControllerNotificationList input, ControllerNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::ActionNotificationList input, ActionNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::RobotEventNotificationList input, RobotEventNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::NetworkNotificationList input, NetworkNotificationList *output)
{ 
	output->clear_notifications();
	for(int i = 0; i < input.notifications.size(); i++)
	{
		ToProtoData(input.notifications[i], output->add_notifications());
	}

	return 0;
}
int ToProtoData(kortex_driver::MappingHandle input, MappingHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_driver::SafetyEvent input, SafetyEvent *output)
{
	ToProtoData(input.safety_handle, output->mutable_safety_handle());

	return 0;
}
int ToProtoData(kortex_driver::ControllerEvent input, ControllerEvent *output)
{
	output->set_input_type((Kinova::Api::Base::ControllerInputType)input.input_type);
	output->set_behavior((Kinova::Api::Base::ControllerBehavior)input.behavior);
	output->set_input_identifier(input.input_identifier);

	return 0;
}
int ToProtoData(kortex_driver::GpioEvent input, GpioEvent *output)
{
	output->set_gpio_state((Kinova::Api::Base::GpioState)input.gpio_state);
	output->set_device_identifier(input.device_identifier);

	return 0;
}
int ToProtoData(kortex_driver::MapEvent input, MapEvent *output)
{
	output->set_name(input.name);

	return 0;
}
int ToProtoData(kortex_driver::MapElement input, MapElement *output)
{
	ToProtoData(input.event, output->mutable_event());
	ToProtoData(input.action, output->mutable_action());

	return 0;
}
int ToProtoData(kortex_driver::ActivateMapHandle input, ActivateMapHandle *output)
{
	ToProtoData(input.mapping_handle, output->mutable_mapping_handle());
	ToProtoData(input.map_group_handle, output->mutable_map_group_handle());
	ToProtoData(input.map_handle, output->mutable_map_handle());

	return 0;
}
int ToProtoData(kortex_driver::Map input, Map *output)
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
int ToProtoData(kortex_driver::MapHandle input, MapHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_driver::MapList input, MapList *output)
{ 
	output->clear_map_list();
	for(int i = 0; i < input.map_list.size(); i++)
	{
		ToProtoData(input.map_list[i], output->add_map_list());
	}

	return 0;
}
int ToProtoData(kortex_driver::MapGroupHandle input, MapGroupHandle *output)
{
	output->set_identifier(input.identifier);
	output->set_permission(input.permission);

	return 0;
}
int ToProtoData(kortex_driver::MapGroup input, MapGroup *output)
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
int ToProtoData(kortex_driver::MapGroupList input, MapGroupList *output)
{ 
	output->clear_map_groups();
	for(int i = 0; i < input.map_groups.size(); i++)
	{
		ToProtoData(input.map_groups[i], output->add_map_groups());
	}

	return 0;
}
int ToProtoData(kortex_driver::Mapping input, Mapping *output)
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
int ToProtoData(kortex_driver::MappingList input, MappingList *output)
{ 
	output->clear_mappings();
	for(int i = 0; i < input.mappings.size(); i++)
	{
		ToProtoData(input.mappings[i], output->add_mappings());
	}

	return 0;
}
int ToProtoData(kortex_driver::TransformationMatrix input, TransformationMatrix *output)
{
	ToProtoData(input.r0, output->mutable_r0());
	ToProtoData(input.r1, output->mutable_r1());
	ToProtoData(input.r2, output->mutable_r2());
	ToProtoData(input.r3, output->mutable_r3());

	return 0;
}
int ToProtoData(kortex_driver::TransformationRow input, TransformationRow *output)
{
	output->set_c0(input.c0);
	output->set_c1(input.c1);
	output->set_c2(input.c2);
	output->set_c3(input.c3);

	return 0;
}
int ToProtoData(kortex_driver::Pose input, Pose *output)
{
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	output->set_theta_x(input.theta_x);
	output->set_theta_y(input.theta_y);
	output->set_theta_z(input.theta_z);

	return 0;
}
int ToProtoData(kortex_driver::Position input, Position *output)
{
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);

	return 0;
}
int ToProtoData(kortex_driver::Orientation input, Orientation *output)
{
	output->set_theta_x(input.theta_x);
	output->set_theta_y(input.theta_y);
	output->set_theta_z(input.theta_z);

	return 0;
}
int ToProtoData(kortex_driver::CartesianSpeed input, CartesianSpeed *output)
{
	output->set_translation(input.translation);
	output->set_orientation(input.orientation);

	return 0;
}
int ToProtoData(kortex_driver::CartesianTrajectoryConstraint input, CartesianTrajectoryConstraint *output)
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
int ToProtoData(kortex_driver::JointTrajectoryConstraint input, JointTrajectoryConstraint *output)
{
	output->set_type((Kinova::Api::Base::JointTrajectoryConstraintType)input.type);
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_driver::Twist input, Twist *output)
{
	output->set_linear_x(input.linear_x);
	output->set_linear_y(input.linear_y);
	output->set_linear_z(input.linear_z);
	output->set_angular_x(input.angular_x);
	output->set_angular_y(input.angular_y);
	output->set_angular_z(input.angular_z);

	return 0;
}
int ToProtoData(kortex_driver::Admittance input, Admittance *output)
{
	output->set_admittance_mode((Kinova::Api::Base::AdmittanceMode)input.admittance_mode);

	return 0;
}
int ToProtoData(kortex_driver::CartesianReferenceFrameRequest input, CartesianReferenceFrameRequest *output)
{
	output->set_reference_frame((Kinova::Api::Base::CartesianReferenceFrame)input.reference_frame);

	return 0;
}
int ToProtoData(kortex_driver::ConstrainedPose input, ConstrainedPose *output)
{
	ToProtoData(input.target_pose, output->mutable_target_pose());
	ToProtoData(input.constraint, output->mutable_constraint());

	return 0;
}
int ToProtoData(kortex_driver::ConstrainedPosition input, ConstrainedPosition *output)
{
	ToProtoData(input.target_position, output->mutable_target_position());
	ToProtoData(input.constraint, output->mutable_constraint());

	return 0;
}
int ToProtoData(kortex_driver::ConstrainedOrientation input, ConstrainedOrientation *output)
{
	ToProtoData(input.target_orientation, output->mutable_target_orientation());
	ToProtoData(input.constraint, output->mutable_constraint());

	return 0;
}
int ToProtoData(kortex_driver::TwistCommand input, TwistCommand *output)
{
	output->set_mode((Kinova::Api::Base::TwistMode)input.mode);
	ToProtoData(input.twist, output->mutable_twist());
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_driver::ConstrainedJointAngles input, ConstrainedJointAngles *output)
{
	ToProtoData(input.joint_angles, output->mutable_joint_angles());
	ToProtoData(input.constraint, output->mutable_constraint());

	return 0;
}
int ToProtoData(kortex_driver::ConstrainedJointAngle input, ConstrainedJointAngle *output)
{
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value);
	ToProtoData(input.constraint, output->mutable_constraint());

	return 0;
}
int ToProtoData(kortex_driver::JointAngles input, JointAngles *output)
{ 
	output->clear_joint_angles();
	for(int i = 0; i < input.joint_angles.size(); i++)
	{
		ToProtoData(input.joint_angles[i], output->add_joint_angles());
	}

	return 0;
}
int ToProtoData(kortex_driver::JointAngle input, JointAngle *output)
{
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_driver::JointSpeeds input, JointSpeeds *output)
{ 
	output->clear_joint_speeds();
	for(int i = 0; i < input.joint_speeds.size(); i++)
	{
		ToProtoData(input.joint_speeds[i], output->add_joint_speeds());
	}
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_driver::JointSpeed input, JointSpeed *output)
{
	output->set_joint_identifier(input.joint_identifier);
	output->set_value(input.value);
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_driver::GripperCommand input, GripperCommand *output)
{
	output->set_mode((Kinova::Api::Base::GripperMode)input.mode);
	ToProtoData(input.gripper, output->mutable_gripper());
	output->set_duration(input.duration);

	return 0;
}
int ToProtoData(kortex_driver::GripperRequest input, GripperRequest *output)
{
	output->set_mode((Kinova::Api::Base::GripperMode)input.mode);

	return 0;
}
int ToProtoData(kortex_driver::Gripper input, Gripper *output)
{ 
	output->clear_finger();
	for(int i = 0; i < input.finger.size(); i++)
	{
		ToProtoData(input.finger[i], output->add_finger());
	}

	return 0;
}
int ToProtoData(kortex_driver::Finger input, Finger *output)
{
	output->set_finger_identifier(input.finger_identifier);
	output->set_value(input.value);

	return 0;
}
int ToProtoData(kortex_driver::SystemTime input, SystemTime *output)
{
	output->set_sec(input.sec);
	output->set_min(input.min);
	output->set_hour(input.hour);
	output->set_mday(input.mday);
	output->set_mon(input.mon);
	output->set_year(input.year);

	return 0;
}
int ToProtoData(kortex_driver::ActuatorInformation input, ActuatorInformation *output)
{
	output->set_count(input.count);

	return 0;
}
int ToProtoData(kortex_driver::ArmStateInformation input, ArmStateInformation *output)
{
	output->set_active_state((Kinova::Api::Common::ArmState)input.active_state);
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
int ToProtoData(kortex_driver::ArmStateNotification input, ArmStateNotification *output)
{
	output->set_active_state((Kinova::Api::Common::ArmState)input.active_state);
	ToProtoData(input.timestamp, output->mutable_timestamp());
	ToProtoData(input.connection, output->mutable_connection());

	return 0;
}
