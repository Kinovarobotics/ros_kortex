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
 
#include "base_ros_converter.h"

#include "common_ros_converter.h"


int ToRosData(FullUserProfile input, kortex_driver::FullUserProfile &output)
{
	ToRosData(input.user_profile(), output.user_profile);
	output.password = input.password();
	
	return 0;
}
int ToRosData(UserProfile input, kortex_driver::UserProfile &output)
{
	ToRosData(input.handle(), output.handle);
	output.username = input.username();
	output.firstname = input.firstname();
	output.lastname = input.lastname();
	output.application_data = input.application_data();
	
	return 0;
}
int ToRosData(UserProfileList input, kortex_driver::UserProfileList &output)
{ 
	output.user_profiles.clear();
	for(int i = 0; i < input.user_profiles_size(); i++)
	{
		kortex_driver::UserProfile temp;
		ToRosData(input.user_profiles(i), temp);
		output.user_profiles.push_back(temp);
	}
	
	return 0;
}
int ToRosData(UserList input, kortex_driver::UserList &output)
{ 
	output.user_handles.clear();
	for(int i = 0; i < input.user_handles_size(); i++)
	{
		kortex_driver::UserProfileHandle temp;
		ToRosData(input.user_handles(i), temp);
		output.user_handles.push_back(temp);
	}
	
	return 0;
}
int ToRosData(PasswordChange input, kortex_driver::PasswordChange &output)
{
	ToRosData(input.handle(), output.handle);
	output.old_password = input.old_password();
	output.new_password = input.new_password();
	
	return 0;
}
int ToRosData(SequenceHandle input, kortex_driver::SequenceHandle &output)
{
	output.identifier = input.identifier();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(AdvancedSequenceHandle input, kortex_driver::AdvancedSequenceHandle &output)
{
	ToRosData(input.handle(), output.handle);
	output.in_loop = input.in_loop();
	
	return 0;
}
int ToRosData(SequenceTaskHandle input, kortex_driver::SequenceTaskHandle &output)
{
	ToRosData(input.sequence_handle(), output.sequence_handle);
	output.task_index = input.task_index();
	
	return 0;
}
int ToRosData(SequenceTask input, kortex_driver::SequenceTask &output)
{
	output.group_identifier = input.group_identifier();
	ToRosData(input.action(), output.action);
	output.application_data = input.application_data();
	
	return 0;
}
int ToRosData(Sequence input, kortex_driver::Sequence &output)
{
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.application_data = input.application_data(); 
	output.tasks.clear();
	for(int i = 0; i < input.tasks_size(); i++)
	{
		kortex_driver::SequenceTask temp;
		ToRosData(input.tasks(i), temp);
		output.tasks.push_back(temp);
	}
	
	return 0;
}
int ToRosData(SequenceList input, kortex_driver::SequenceList &output)
{ 
	output.sequence_list.clear();
	for(int i = 0; i < input.sequence_list_size(); i++)
	{
		kortex_driver::Sequence temp;
		ToRosData(input.sequence_list(i), temp);
		output.sequence_list.push_back(temp);
	}
	
	return 0;
}
int ToRosData(AppendActionInformation input, kortex_driver::AppendActionInformation &output)
{
	ToRosData(input.sequence_handle(), output.sequence_handle);
	ToRosData(input.action(), output.action);
	
	return 0;
}
int ToRosData(ActionHandle input, kortex_driver::ActionHandle &output)
{
	output.identifier = input.identifier();
	output.action_type = input.action_type();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(RequestedActionType input, kortex_driver::RequestedActionType &output)
{
	output.action_type = input.action_type();
	
	return 0;
}
int ToRosData(Action input, kortex_driver::Action &output)
{
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.application_data = input.application_data();
	
	
	auto oneof_type = input.action_parameters_case();

	switch(oneof_type)
	{
		case Action::kSendTwistCommand:
		{
			kortex_driver::TwistCommand temp;
			ToRosData(input.send_twist_command(), temp);
			output.oneof_action_parameters.send_twist_command.push_back(temp);
			break;
		}
	
		case Action::kSendJointSpeeds:
		{
			kortex_driver::JointSpeeds temp;
			ToRosData(input.send_joint_speeds(), temp);
			output.oneof_action_parameters.send_joint_speeds.push_back(temp);
			break;
		}
	
		case Action::kReachPose:
		{
			kortex_driver::ConstrainedPose temp;
			ToRosData(input.reach_pose(), temp);
			output.oneof_action_parameters.reach_pose.push_back(temp);
			break;
		}
	
		case Action::kReachJointAngles:
		{
			kortex_driver::ConstrainedJointAngles temp;
			ToRosData(input.reach_joint_angles(), temp);
			output.oneof_action_parameters.reach_joint_angles.push_back(temp);
			break;
		}
	
		case Action::kToggleAdmittanceMode:
		{
			output.oneof_action_parameters.toggle_admittance_mode.push_back(input.toggle_admittance_mode());
	
			break;
		}
	
		case Action::kSwitchControlMapping:
		{
			kortex_driver::SwitchControlMapping temp;
			ToRosData(input.switch_control_mapping(), temp);
			output.oneof_action_parameters.switch_control_mapping.push_back(temp);
			break;
		}
	
		case Action::kNavigateJoints:
		{
			output.oneof_action_parameters.navigate_joints.push_back(input.navigate_joints());
	
			break;
		}
	
		case Action::kNavigateMappings:
		{
			output.oneof_action_parameters.navigate_mappings.push_back(input.navigate_mappings());
	
			break;
		}
	
		case Action::kChangeTwist:
		{
			kortex_driver::ChangeTwist temp;
			ToRosData(input.change_twist(), temp);
			output.oneof_action_parameters.change_twist.push_back(temp);
			break;
		}
	
		case Action::kChangeJointSpeeds:
		{
			kortex_driver::ChangeJointSpeeds temp;
			ToRosData(input.change_joint_speeds(), temp);
			output.oneof_action_parameters.change_joint_speeds.push_back(temp);
			break;
		}
	
		case Action::kApplyEmergencyStop:
		{
			kortex_driver::EmergencyStop temp;
			ToRosData(input.apply_emergency_stop(), temp);
			output.oneof_action_parameters.apply_emergency_stop.push_back(temp);
			break;
		}
	
		case Action::kClearFaults:
		{
			kortex_driver::Faults temp;
			ToRosData(input.clear_faults(), temp);
			output.oneof_action_parameters.clear_faults.push_back(temp);
			break;
		}
	
		case Action::kDelay:
		{
			kortex_driver::Delay temp;
			ToRosData(input.delay(), temp);
			output.oneof_action_parameters.delay.push_back(temp);
			break;
		}
	
		case Action::kExecuteAction:
		{
			kortex_driver::ActionHandle temp;
			ToRosData(input.execute_action(), temp);
			output.oneof_action_parameters.execute_action.push_back(temp);
			break;
		}
	
		case Action::kSendGripperCommand:
		{
			kortex_driver::GripperCommand temp;
			ToRosData(input.send_gripper_command(), temp);
			output.oneof_action_parameters.send_gripper_command.push_back(temp);
			break;
		}
	
		case Action::kStopAction:
		{
			kortex_driver::Stop temp;
			ToRosData(input.stop_action(), temp);
			output.oneof_action_parameters.stop_action.push_back(temp);
			break;
		}
	
	}
	return 0;
}
int ToRosData(SwitchControlMapping input, kortex_driver::SwitchControlMapping &output)
{
	output.controller_identifier = input.controller_identifier();
	ToRosData(input.map_group_handle(), output.map_group_handle);
	ToRosData(input.map_handle(), output.map_handle);
	
	return 0;
}
int ToRosData(ChangeTwist input, kortex_driver::ChangeTwist &output)
{
	output.linear = input.linear();
	output.angular = input.angular();
	
	return 0;
}
int ToRosData(ChangeJointSpeeds input, kortex_driver::ChangeJointSpeeds &output)
{
	ToRosData(input.joint_speeds(), output.joint_speeds);
	
	return 0;
}
int ToRosData(EmergencyStop input, kortex_driver::EmergencyStop &output)
{
	
	return 0;
}
int ToRosData(Faults input, kortex_driver::Faults &output)
{
	
	return 0;
}
int ToRosData(Delay input, kortex_driver::Delay &output)
{
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(Stop input, kortex_driver::Stop &output)
{
	
	return 0;
}
int ToRosData(ActionList input, kortex_driver::ActionList &output)
{ 
	output.action_list.clear();
	for(int i = 0; i < input.action_list_size(); i++)
	{
		kortex_driver::Action temp;
		ToRosData(input.action_list(i), temp);
		output.action_list.push_back(temp);
	}
	
	return 0;
}
int ToRosData(Timeout input, kortex_driver::Timeout &output)
{
	output.value = input.value();
	
	return 0;
}
int ToRosData(Ssid input, kortex_driver::Ssid &output)
{
	output.identifier = input.identifier();
	
	return 0;
}
int ToRosData(CommunicationInterfaceConfiguration input, kortex_driver::CommunicationInterfaceConfiguration &output)
{
	output.type = input.type();
	output.enable = input.enable();
	
	return 0;
}
int ToRosData(NetworkHandle input, kortex_driver::NetworkHandle &output)
{
	output.type = input.type();
	
	return 0;
}
int ToRosData(IPv4Configuration input, kortex_driver::IPv4Configuration &output)
{
	output.ip_address = input.ip_address();
	output.subnet_mask = input.subnet_mask();
	output.default_gateway = input.default_gateway();
	output.dhcp_enabled = input.dhcp_enabled();
	
	return 0;
}
int ToRosData(IPv4Information input, kortex_driver::IPv4Information &output)
{
	output.ip_address = input.ip_address();
	output.subnet_mask = input.subnet_mask();
	output.default_gateway = input.default_gateway();
	
	return 0;
}
int ToRosData(FullIPv4Configuration input, kortex_driver::FullIPv4Configuration &output)
{
	ToRosData(input.handle(), output.handle);
	ToRosData(input.ipv4_configuration(), output.ipv4_configuration);
	
	return 0;
}
int ToRosData(WifiInformation input, kortex_driver::WifiInformation &output)
{
	ToRosData(input.ssid(), output.ssid);
	output.security_type = input.security_type();
	output.encryption_type = input.encryption_type();
	output.signal_quality = input.signal_quality();
	output.signal_strength = input.signal_strength();
	output.frequency = input.frequency();
	output.channel = input.channel();
	
	return 0;
}
int ToRosData(WifiInformationList input, kortex_driver::WifiInformationList &output)
{ 
	output.wifi_information_list.clear();
	for(int i = 0; i < input.wifi_information_list_size(); i++)
	{
		kortex_driver::WifiInformation temp;
		ToRosData(input.wifi_information_list(i), temp);
		output.wifi_information_list.push_back(temp);
	}
	
	return 0;
}
int ToRosData(WifiConfiguration input, kortex_driver::WifiConfiguration &output)
{
	ToRosData(input.ssid(), output.ssid);
	output.security_key = input.security_key();
	output.connect_automatically = input.connect_automatically();
	
	return 0;
}
int ToRosData(WifiConfigurationList input, kortex_driver::WifiConfigurationList &output)
{ 
	output.wifi_configuration_list.clear();
	for(int i = 0; i < input.wifi_configuration_list_size(); i++)
	{
		kortex_driver::WifiConfiguration temp;
		ToRosData(input.wifi_configuration_list(i), temp);
		output.wifi_configuration_list.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ProtectionZoneHandle input, kortex_driver::ProtectionZoneHandle &output)
{
	output.identifier = input.identifier();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(RotationMatrixRow input, kortex_driver::RotationMatrixRow &output)
{
	output.column1 = input.column1();
	output.column2 = input.column2();
	output.column3 = input.column3();
	
	return 0;
}
int ToRosData(RotationMatrix input, kortex_driver::RotationMatrix &output)
{
	ToRosData(input.row1(), output.row1);
	ToRosData(input.row2(), output.row2);
	ToRosData(input.row3(), output.row3);
	
	return 0;
}
int ToRosData(Point input, kortex_driver::Point &output)
{
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();
	
	return 0;
}
int ToRosData(ZoneShape input, kortex_driver::ZoneShape &output)
{
	output.shape_type = input.shape_type();
	ToRosData(input.origin(), output.origin);
	ToRosData(input.orientation(), output.orientation); 
	
	output.dimensions.clear();
	for(int i = 0; i < input.dimensions_size(); i++)
	{
		output.dimensions.push_back(input.dimensions(i));
	}
	output.envelope_thickness = input.envelope_thickness();
	
	return 0;
}
int ToRosData(ProtectionZone input, kortex_driver::ProtectionZone &output)
{
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.application_data = input.application_data();
	output.is_enabled = input.is_enabled();
	ToRosData(input.shape(), output.shape); 
	output.limitations.clear();
	for(int i = 0; i < input.limitations_size(); i++)
	{
		kortex_driver::CartesianLimitation temp;
		ToRosData(input.limitations(i), temp);
		output.limitations.push_back(temp);
	} 
	output.envelope_limitations.clear();
	for(int i = 0; i < input.envelope_limitations_size(); i++)
	{
		kortex_driver::CartesianLimitation temp;
		ToRosData(input.envelope_limitations(i), temp);
		output.envelope_limitations.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ProtectionZoneList input, kortex_driver::ProtectionZoneList &output)
{ 
	output.protection_zones.clear();
	for(int i = 0; i < input.protection_zones_size(); i++)
	{
		kortex_driver::ProtectionZone temp;
		ToRosData(input.protection_zones(i), temp);
		output.protection_zones.push_back(temp);
	}
	
	return 0;
}
int ToRosData(LimitationTypeIdentifier input, kortex_driver::LimitationTypeIdentifier &output)
{
	output.type = input.type();
	
	return 0;
}
int ToRosData(CartesianLimitation input, kortex_driver::CartesianLimitation &output)
{
	output.type = input.type();
	output.translation = input.translation();
	output.orientation = input.orientation();
	
	return 0;
}
int ToRosData(CartesianLimitationList input, kortex_driver::CartesianLimitationList &output)
{ 
	output.limitations.clear();
	for(int i = 0; i < input.limitations_size(); i++)
	{
		kortex_driver::CartesianLimitation temp;
		ToRosData(input.limitations(i), temp);
		output.limitations.push_back(temp);
	}
	
	return 0;
}
int ToRosData(JointLimitationValue input, kortex_driver::JointLimitationValue &output)
{
	output.type = input.type();
	output.value = input.value();
	
	return 0;
}
int ToRosData(JointLimitationValueList input, kortex_driver::JointLimitationValueList &output)
{ 
	output.joint_limitation_values.clear();
	for(int i = 0; i < input.joint_limitation_values_size(); i++)
	{
		kortex_driver::JointLimitationValue temp;
		ToRosData(input.joint_limitation_values(i), temp);
		output.joint_limitation_values.push_back(temp);
	}
	
	return 0;
}
int ToRosData(JointLimitation input, kortex_driver::JointLimitation &output)
{
	output.device_identifier = input.device_identifier();
	ToRosData(input.limitation_value(), output.limitation_value);
	
	return 0;
}
int ToRosData(JointLimitationTypeIdentifier input, kortex_driver::JointLimitationTypeIdentifier &output)
{
	output.device_identifier = input.device_identifier();
	output.type = input.type();
	
	return 0;
}
int ToRosData(Query input, kortex_driver::Query &output)
{
	ToRosData(input.start_timestamp(), output.start_timestamp);
	ToRosData(input.end_timestamp(), output.end_timestamp);
	output.username = input.username();
	
	return 0;
}
int ToRosData(ConfigurationChangeNotification input, kortex_driver::ConfigurationChangeNotification &output)
{
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(MappingInfoNotification input, kortex_driver::MappingInfoNotification &output)
{
	output.controller_identifier = input.controller_identifier();
	ToRosData(input.active_map_handle(), output.active_map_handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ControlModeInformation input, kortex_driver::ControlModeInformation &output)
{
	output.mode = input.mode();
	
	return 0;
}
int ToRosData(ControlModeNotification input, kortex_driver::ControlModeNotification &output)
{
	output.control_mode = input.control_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ServoingModeInformation input, kortex_driver::ServoingModeInformation &output)
{
	output.servoing_mode = input.servoing_mode();
	
	return 0;
}
int ToRosData(OperatingModeInformation input, kortex_driver::OperatingModeInformation &output)
{
	output.operating_mode = input.operating_mode();
	ToRosData(input.device_handle(), output.device_handle);
	
	return 0;
}
int ToRosData(OperatingModeNotification input, kortex_driver::OperatingModeNotification &output)
{
	output.operating_mode = input.operating_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	ToRosData(input.device_handle(), output.device_handle);
	
	return 0;
}
int ToRosData(ServoingModeNotification input, kortex_driver::ServoingModeNotification &output)
{
	output.servoing_mode = input.servoing_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(SequenceInfoNotification input, kortex_driver::SequenceInfoNotification &output)
{
	output.event_identifier = input.event_identifier();
	ToRosData(input.sequence_handle(), output.sequence_handle);
	output.task_index = input.task_index();
	output.group_identifier = input.group_identifier();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	output.abort_details = input.abort_details();
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(SequenceInformation input, kortex_driver::SequenceInformation &output)
{
	output.event_identifier = input.event_identifier();
	output.task_index = input.task_index();
	output.task_identifier = input.task_identifier();
	
	return 0;
}
int ToRosData(ProtectionZoneNotification input, kortex_driver::ProtectionZoneNotification &output)
{
	output.event = input.event();
	ToRosData(input.handle(), output.handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ProtectionZoneInformation input, kortex_driver::ProtectionZoneInformation &output)
{
	output.event = input.event();
	
	return 0;
}
int ToRosData(UserNotification input, kortex_driver::UserNotification &output)
{
	output.user_event = input.user_event();
	ToRosData(input.modified_user(), output.modified_user);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ControllerHandle input, kortex_driver::ControllerHandle &output)
{
	output.type = input.type();
	output.controller_identifier = input.controller_identifier();
	
	return 0;
}
int ToRosData(ControllerElementHandle input, kortex_driver::ControllerElementHandle &output)
{
	ToRosData(input.controller_handle(), output.controller_handle);
	
	
	auto oneof_type = input.identifier_case();

	switch(oneof_type)
	{
		case ControllerElementHandle::kButton:
		{
			break;
		}
	
		case ControllerElementHandle::kAxis:
		{
			break;
		}
	
	}
	return 0;
}
int ToRosData(ControllerNotification input, kortex_driver::ControllerNotification &output)
{
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ControllerList input, kortex_driver::ControllerList &output)
{ 
	output.handles.clear();
	for(int i = 0; i < input.handles_size(); i++)
	{
		kortex_driver::ControllerHandle temp;
		ToRosData(input.handles(i), temp);
		output.handles.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ControllerState input, kortex_driver::ControllerState &output)
{
	ToRosData(input.handle(), output.handle);
	output.event_type = input.event_type();
	
	return 0;
}
int ToRosData(ControllerElementState input, kortex_driver::ControllerElementState &output)
{
	ToRosData(input.handle(), output.handle);
	output.event_type = input.event_type();
	output.axis_value = input.axis_value();
	
	return 0;
}
int ToRosData(ActionNotification input, kortex_driver::ActionNotification &output)
{
	output.action_event = input.action_event();
	ToRosData(input.handle(), output.handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	output.abort_details = input.abort_details();
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ActionExecutionState input, kortex_driver::ActionExecutionState &output)
{
	output.action_event = input.action_event();
	ToRosData(input.handle(), output.handle);
	
	return 0;
}
int ToRosData(RobotEventNotification input, kortex_driver::RobotEventNotification &output)
{
	output.event = input.event();
	ToRosData(input.handle(), output.handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(FactoryNotification input, kortex_driver::FactoryNotification &output)
{
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(NetworkNotification input, kortex_driver::NetworkNotification &output)
{
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ConfigurationChangeNotificationList input, kortex_driver::ConfigurationChangeNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::ConfigurationChangeNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(MappingInfoNotificationList input, kortex_driver::MappingInfoNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::MappingInfoNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ControlModeNotificationList input, kortex_driver::ControlModeNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::ControlModeNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(OperatingModeNotificationList input, kortex_driver::OperatingModeNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::OperatingModeNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ServoingModeNotificationList input, kortex_driver::ServoingModeNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::ServoingModeNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(SequenceInfoNotificationList input, kortex_driver::SequenceInfoNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::SequenceInfoNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ProtectionZoneNotificationList input, kortex_driver::ProtectionZoneNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::ProtectionZoneNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(UserNotificationList input, kortex_driver::UserNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::UserNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(SafetyNotificationList input, kortex_driver::SafetyNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::SafetyNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ControllerNotificationList input, kortex_driver::ControllerNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::ControllerNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(ActionNotificationList input, kortex_driver::ActionNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::ActionNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(RobotEventNotificationList input, kortex_driver::RobotEventNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::RobotEventNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(NetworkNotificationList input, kortex_driver::NetworkNotificationList &output)
{ 
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		kortex_driver::NetworkNotification temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}
	
	return 0;
}
int ToRosData(MappingHandle input, kortex_driver::MappingHandle &output)
{
	output.identifier = input.identifier();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(SafetyEvent input, kortex_driver::SafetyEvent &output)
{
	ToRosData(input.safety_handle(), output.safety_handle);
	
	return 0;
}
int ToRosData(ControllerEvent input, kortex_driver::ControllerEvent &output)
{
	output.input_type = input.input_type();
	output.behavior = input.behavior();
	output.input_identifier = input.input_identifier();
	
	return 0;
}
int ToRosData(GpioEvent input, kortex_driver::GpioEvent &output)
{
	output.gpio_state = input.gpio_state();
	output.device_identifier = input.device_identifier();
	
	return 0;
}
int ToRosData(MapEvent input, kortex_driver::MapEvent &output)
{
	output.name = input.name();
	
	return 0;
}
int ToRosData(MapElement input, kortex_driver::MapElement &output)
{
	ToRosData(input.event(), output.event);
	ToRosData(input.action(), output.action);
	
	return 0;
}
int ToRosData(ActivateMapHandle input, kortex_driver::ActivateMapHandle &output)
{
	ToRosData(input.mapping_handle(), output.mapping_handle);
	ToRosData(input.map_group_handle(), output.map_group_handle);
	ToRosData(input.map_handle(), output.map_handle);
	
	return 0;
}
int ToRosData(Map input, kortex_driver::Map &output)
{
	ToRosData(input.handle(), output.handle);
	output.name = input.name(); 
	output.elements.clear();
	for(int i = 0; i < input.elements_size(); i++)
	{
		kortex_driver::MapElement temp;
		ToRosData(input.elements(i), temp);
		output.elements.push_back(temp);
	}
	
	return 0;
}
int ToRosData(MapHandle input, kortex_driver::MapHandle &output)
{
	output.identifier = input.identifier();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(MapList input, kortex_driver::MapList &output)
{ 
	output.map_list.clear();
	for(int i = 0; i < input.map_list_size(); i++)
	{
		kortex_driver::Map temp;
		ToRosData(input.map_list(i), temp);
		output.map_list.push_back(temp);
	}
	
	return 0;
}
int ToRosData(MapGroupHandle input, kortex_driver::MapGroupHandle &output)
{
	output.identifier = input.identifier();
	output.permission = input.permission();
	
	return 0;
}
int ToRosData(MapGroup input, kortex_driver::MapGroup &output)
{
	ToRosData(input.group_handle(), output.group_handle);
	output.name = input.name();
	ToRosData(input.related_mapping_handle(), output.related_mapping_handle);
	ToRosData(input.parent_group_handle(), output.parent_group_handle); 
	output.children_map_group_handles.clear();
	for(int i = 0; i < input.children_map_group_handles_size(); i++)
	{
		kortex_driver::MapGroupHandle temp;
		ToRosData(input.children_map_group_handles(i), temp);
		output.children_map_group_handles.push_back(temp);
	} 
	output.map_handles.clear();
	for(int i = 0; i < input.map_handles_size(); i++)
	{
		kortex_driver::MapHandle temp;
		ToRosData(input.map_handles(i), temp);
		output.map_handles.push_back(temp);
	}
	output.application_data = input.application_data();
	
	return 0;
}
int ToRosData(MapGroupList input, kortex_driver::MapGroupList &output)
{ 
	output.map_groups.clear();
	for(int i = 0; i < input.map_groups_size(); i++)
	{
		kortex_driver::MapGroup temp;
		ToRosData(input.map_groups(i), temp);
		output.map_groups.push_back(temp);
	}
	
	return 0;
}
int ToRosData(Mapping input, kortex_driver::Mapping &output)
{
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.controller_identifier = input.controller_identifier();
	ToRosData(input.active_map_group_handle(), output.active_map_group_handle); 
	output.map_group_handles.clear();
	for(int i = 0; i < input.map_group_handles_size(); i++)
	{
		kortex_driver::MapGroupHandle temp;
		ToRosData(input.map_group_handles(i), temp);
		output.map_group_handles.push_back(temp);
	}
	ToRosData(input.active_map_handle(), output.active_map_handle); 
	output.map_handles.clear();
	for(int i = 0; i < input.map_handles_size(); i++)
	{
		kortex_driver::MapHandle temp;
		ToRosData(input.map_handles(i), temp);
		output.map_handles.push_back(temp);
	}
	output.application_data = input.application_data();
	
	return 0;
}
int ToRosData(MappingList input, kortex_driver::MappingList &output)
{ 
	output.mappings.clear();
	for(int i = 0; i < input.mappings_size(); i++)
	{
		kortex_driver::Mapping temp;
		ToRosData(input.mappings(i), temp);
		output.mappings.push_back(temp);
	}
	
	return 0;
}
int ToRosData(TransformationMatrix input, kortex_driver::TransformationMatrix &output)
{
	ToRosData(input.r0(), output.r0);
	ToRosData(input.r1(), output.r1);
	ToRosData(input.r2(), output.r2);
	ToRosData(input.r3(), output.r3);
	
	return 0;
}
int ToRosData(TransformationRow input, kortex_driver::TransformationRow &output)
{
	output.c0 = input.c0();
	output.c1 = input.c1();
	output.c2 = input.c2();
	output.c3 = input.c3();
	
	return 0;
}
int ToRosData(Pose input, kortex_driver::Pose &output)
{
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();
	output.theta_x = input.theta_x();
	output.theta_y = input.theta_y();
	output.theta_z = input.theta_z();
	
	return 0;
}
int ToRosData(Position input, kortex_driver::Position &output)
{
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();
	
	return 0;
}
int ToRosData(Orientation input, kortex_driver::Orientation &output)
{
	output.theta_x = input.theta_x();
	output.theta_y = input.theta_y();
	output.theta_z = input.theta_z();
	
	return 0;
}
int ToRosData(CartesianSpeed input, kortex_driver::CartesianSpeed &output)
{
	output.translation = input.translation();
	output.orientation = input.orientation();
	
	return 0;
}
int ToRosData(CartesianTrajectoryConstraint input, kortex_driver::CartesianTrajectoryConstraint &output)
{
	
	
	auto oneof_type = input.type_case();

	switch(oneof_type)
	{
		case CartesianTrajectoryConstraint::kSpeed:
		{
			kortex_driver::CartesianSpeed temp;
			ToRosData(input.speed(), temp);
			output.oneof_type.speed.push_back(temp);
			break;
		}
	
		case CartesianTrajectoryConstraint::kDuration:
		{
			break;
		}
	
	}
	return 0;
}
int ToRosData(JointTrajectoryConstraint input, kortex_driver::JointTrajectoryConstraint &output)
{
	output.type = input.type();
	output.value = input.value();
	
	return 0;
}
int ToRosData(Twist input, kortex_driver::Twist &output)
{
	output.linear_x = input.linear_x();
	output.linear_y = input.linear_y();
	output.linear_z = input.linear_z();
	output.angular_x = input.angular_x();
	output.angular_y = input.angular_y();
	output.angular_z = input.angular_z();
	
	return 0;
}
int ToRosData(Admittance input, kortex_driver::Admittance &output)
{
	output.admittance_mode = input.admittance_mode();
	
	return 0;
}
int ToRosData(CartesianReferenceFrameRequest input, kortex_driver::CartesianReferenceFrameRequest &output)
{
	output.reference_frame = input.reference_frame();
	
	return 0;
}
int ToRosData(ConstrainedPose input, kortex_driver::ConstrainedPose &output)
{
	ToRosData(input.target_pose(), output.target_pose);
	ToRosData(input.constraint(), output.constraint);
	
	return 0;
}
int ToRosData(ConstrainedPosition input, kortex_driver::ConstrainedPosition &output)
{
	ToRosData(input.target_position(), output.target_position);
	ToRosData(input.constraint(), output.constraint);
	
	return 0;
}
int ToRosData(ConstrainedOrientation input, kortex_driver::ConstrainedOrientation &output)
{
	ToRosData(input.target_orientation(), output.target_orientation);
	ToRosData(input.constraint(), output.constraint);
	
	return 0;
}
int ToRosData(TwistCommand input, kortex_driver::TwistCommand &output)
{
	output.mode = input.mode();
	ToRosData(input.twist(), output.twist);
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(ConstrainedJointAngles input, kortex_driver::ConstrainedJointAngles &output)
{
	ToRosData(input.joint_angles(), output.joint_angles);
	ToRosData(input.constraint(), output.constraint);
	
	return 0;
}
int ToRosData(ConstrainedJointAngle input, kortex_driver::ConstrainedJointAngle &output)
{
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();
	ToRosData(input.constraint(), output.constraint);
	
	return 0;
}
int ToRosData(JointAngles input, kortex_driver::JointAngles &output)
{ 
	output.joint_angles.clear();
	for(int i = 0; i < input.joint_angles_size(); i++)
	{
		kortex_driver::JointAngle temp;
		ToRosData(input.joint_angles(i), temp);
		output.joint_angles.push_back(temp);
	}
	
	return 0;
}
int ToRosData(JointAngle input, kortex_driver::JointAngle &output)
{
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();
	
	return 0;
}
int ToRosData(JointSpeeds input, kortex_driver::JointSpeeds &output)
{ 
	output.joint_speeds.clear();
	for(int i = 0; i < input.joint_speeds_size(); i++)
	{
		kortex_driver::JointSpeed temp;
		ToRosData(input.joint_speeds(i), temp);
		output.joint_speeds.push_back(temp);
	}
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(JointSpeed input, kortex_driver::JointSpeed &output)
{
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(GripperCommand input, kortex_driver::GripperCommand &output)
{
	output.mode = input.mode();
	ToRosData(input.gripper(), output.gripper);
	output.duration = input.duration();
	
	return 0;
}
int ToRosData(GripperRequest input, kortex_driver::GripperRequest &output)
{
	output.mode = input.mode();
	
	return 0;
}
int ToRosData(Gripper input, kortex_driver::Gripper &output)
{ 
	output.finger.clear();
	for(int i = 0; i < input.finger_size(); i++)
	{
		kortex_driver::Finger temp;
		ToRosData(input.finger(i), temp);
		output.finger.push_back(temp);
	}
	
	return 0;
}
int ToRosData(Finger input, kortex_driver::Finger &output)
{
	output.finger_identifier = input.finger_identifier();
	output.value = input.value();
	
	return 0;
}
int ToRosData(SystemTime input, kortex_driver::SystemTime &output)
{
	output.sec = input.sec();
	output.min = input.min();
	output.hour = input.hour();
	output.mday = input.mday();
	output.mon = input.mon();
	output.year = input.year();
	
	return 0;
}
int ToRosData(ActuatorInformation input, kortex_driver::ActuatorInformation &output)
{
	output.count = input.count();
	
	return 0;
}
int ToRosData(ArmStateInformation input, kortex_driver::ArmStateInformation &output)
{
	output.active_state = input.active_state();
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
int ToRosData(ArmStateNotification input, kortex_driver::ArmStateNotification &output)
{
	output.active_state = input.active_state();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.connection(), output.connection);
	
	return 0;
}
