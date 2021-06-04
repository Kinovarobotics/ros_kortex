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
 
#include "kortex_driver/generated/robot/base_ros_converter.h"

int ToRosData(Kinova::Api::Base::GpioConfigurationList input, kortex_driver::GpioConfigurationList &output)
{
	
	output.port_configurations.clear();
	for(int i = 0; i < input.port_configurations_size(); i++)
	{
		decltype(output.port_configurations)::value_type temp;
		ToRosData(input.port_configurations(i), temp);
		output.port_configurations.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::GpioConfiguration input, kortex_driver::Base_GpioConfiguration &output)
{
	
	output.port_number = input.port_number();
	output.pin_configurations.clear();
	for(int i = 0; i < input.pin_configurations_size(); i++)
	{
		decltype(output.pin_configurations)::value_type temp;
		ToRosData(input.pin_configurations(i), temp);
		output.pin_configurations.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::GpioPinConfiguration input, kortex_driver::GpioPinConfiguration &output)
{
	
	output.pin_id = input.pin_id();
	output.pin_property = input.pin_property();
	output.output_enable = input.output_enable();
	output.default_output_value = input.default_output_value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::FullUserProfile input, kortex_driver::FullUserProfile &output)
{
	
	ToRosData(input.user_profile(), output.user_profile);
	output.password = input.password();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::UserProfile input, kortex_driver::UserProfile &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.username = input.username();
	output.firstname = input.firstname();
	output.lastname = input.lastname();
	output.application_data = input.application_data();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::UserProfileList input, kortex_driver::UserProfileList &output)
{
	
	output.user_profiles.clear();
	for(int i = 0; i < input.user_profiles_size(); i++)
	{
		decltype(output.user_profiles)::value_type temp;
		ToRosData(input.user_profiles(i), temp);
		output.user_profiles.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::UserList input, kortex_driver::UserList &output)
{
	
	output.user_handles.clear();
	for(int i = 0; i < input.user_handles_size(); i++)
	{
		decltype(output.user_handles)::value_type temp;
		ToRosData(input.user_handles(i), temp);
		output.user_handles.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::PasswordChange input, kortex_driver::PasswordChange &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.old_password = input.old_password();
	output.new_password = input.new_password();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceHandle input, kortex_driver::SequenceHandle &output)
{
	
	output.identifier = input.identifier();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::AdvancedSequenceHandle input, kortex_driver::AdvancedSequenceHandle &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.in_loop = input.in_loop();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTaskHandle input, kortex_driver::SequenceTaskHandle &output)
{
	
	ToRosData(input.sequence_handle(), output.sequence_handle);
	output.task_index = input.task_index();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTask input, kortex_driver::SequenceTask &output)
{
	
	output.group_identifier = input.group_identifier();
	ToRosData(input.action(), output.action);
	output.application_data = input.application_data();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTasks input, kortex_driver::SequenceTasks &output)
{
	
	output.sequence_tasks.clear();
	for(int i = 0; i < input.sequence_tasks_size(); i++)
	{
		decltype(output.sequence_tasks)::value_type temp;
		ToRosData(input.sequence_tasks(i), temp);
		output.sequence_tasks.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTasksConfiguration input, kortex_driver::SequenceTasksConfiguration &output)
{
	
	ToRosData(input.sequence_task_handle(), output.sequence_task_handle);
	output.sequence_tasks.clear();
	for(int i = 0; i < input.sequence_tasks_size(); i++)
	{
		decltype(output.sequence_tasks)::value_type temp;
		ToRosData(input.sequence_tasks(i), temp);
		output.sequence_tasks.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTaskConfiguration input, kortex_driver::SequenceTaskConfiguration &output)
{
	
	ToRosData(input.sequence_task_handle(), output.sequence_task_handle);
	ToRosData(input.sequence_task(), output.sequence_task);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTasksRange input, kortex_driver::SequenceTasksRange &output)
{
	
	output.first_task_index = input.first_task_index();
	output.second_task_index = input.second_task_index();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceTasksPair input, kortex_driver::SequenceTasksPair &output)
{
	
	ToRosData(input.sequence_handle(), output.sequence_handle);
	output.first_task_index = input.first_task_index();
	output.second_task_index = input.second_task_index();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Sequence input, kortex_driver::Sequence &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.application_data = input.application_data();
	output.tasks.clear();
	for(int i = 0; i < input.tasks_size(); i++)
	{
		decltype(output.tasks)::value_type temp;
		ToRosData(input.tasks(i), temp);
		output.tasks.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceList input, kortex_driver::SequenceList &output)
{
	
	output.sequence_list.clear();
	for(int i = 0; i < input.sequence_list_size(); i++)
	{
		decltype(output.sequence_list)::value_type temp;
		ToRosData(input.sequence_list(i), temp);
		output.sequence_list.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::AppendActionInformation input, kortex_driver::AppendActionInformation &output)
{
	
	ToRosData(input.sequence_handle(), output.sequence_handle);
	ToRosData(input.action(), output.action);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActionHandle input, kortex_driver::ActionHandle &output)
{
	
	output.identifier = input.identifier();
	output.action_type = input.action_type();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::RequestedActionType input, kortex_driver::RequestedActionType &output)
{
	
	output.action_type = input.action_type();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Action input, kortex_driver::Action &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.application_data = input.application_data();

	
	auto oneof_type_action_parameters = input.action_parameters_case();
	switch(oneof_type_action_parameters)
	{ 
	
		case Kinova::Api::Base::Action::kSendTwistCommand:
		{
			decltype(output.oneof_action_parameters.send_twist_command)::value_type temp;
			ToRosData(input.send_twist_command(), temp);
			output.oneof_action_parameters.send_twist_command.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kSendWrenchCommand:
		{
			decltype(output.oneof_action_parameters.send_wrench_command)::value_type temp;
			ToRosData(input.send_wrench_command(), temp);
			output.oneof_action_parameters.send_wrench_command.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kSendJointSpeeds:
		{
			decltype(output.oneof_action_parameters.send_joint_speeds)::value_type temp;
			ToRosData(input.send_joint_speeds(), temp);
			output.oneof_action_parameters.send_joint_speeds.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kReachPose:
		{
			decltype(output.oneof_action_parameters.reach_pose)::value_type temp;
			ToRosData(input.reach_pose(), temp);
			output.oneof_action_parameters.reach_pose.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kReachJointAngles:
		{
			decltype(output.oneof_action_parameters.reach_joint_angles)::value_type temp;
			ToRosData(input.reach_joint_angles(), temp);
			output.oneof_action_parameters.reach_joint_angles.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kToggleAdmittanceMode:
		{
			output.oneof_action_parameters.toggle_admittance_mode.push_back(input.toggle_admittance_mode());
	
			break;
		} 
	
		case Kinova::Api::Base::Action::kSnapshot:
		{
			decltype(output.oneof_action_parameters.snapshot)::value_type temp;
			ToRosData(input.snapshot(), temp);
			output.oneof_action_parameters.snapshot.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kSwitchControlMapping:
		{
			decltype(output.oneof_action_parameters.switch_control_mapping)::value_type temp;
			ToRosData(input.switch_control_mapping(), temp);
			output.oneof_action_parameters.switch_control_mapping.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kNavigateJoints:
		{
			output.oneof_action_parameters.navigate_joints.push_back(input.navigate_joints());
	
			break;
		} 
	
		case Kinova::Api::Base::Action::kNavigateMappings:
		{
			output.oneof_action_parameters.navigate_mappings.push_back(input.navigate_mappings());
	
			break;
		} 
	
		case Kinova::Api::Base::Action::kChangeTwist:
		{
			decltype(output.oneof_action_parameters.change_twist)::value_type temp;
			ToRosData(input.change_twist(), temp);
			output.oneof_action_parameters.change_twist.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kChangeJointSpeeds:
		{
			decltype(output.oneof_action_parameters.change_joint_speeds)::value_type temp;
			ToRosData(input.change_joint_speeds(), temp);
			output.oneof_action_parameters.change_joint_speeds.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kChangeWrench:
		{
			decltype(output.oneof_action_parameters.change_wrench)::value_type temp;
			ToRosData(input.change_wrench(), temp);
			output.oneof_action_parameters.change_wrench.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kApplyEmergencyStop:
		{
			decltype(output.oneof_action_parameters.apply_emergency_stop)::value_type temp;
			ToRosData(input.apply_emergency_stop(), temp);
			output.oneof_action_parameters.apply_emergency_stop.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kClearFaults:
		{
			decltype(output.oneof_action_parameters.clear_faults)::value_type temp;
			ToRosData(input.clear_faults(), temp);
			output.oneof_action_parameters.clear_faults.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kDelay:
		{
			decltype(output.oneof_action_parameters.delay)::value_type temp;
			ToRosData(input.delay(), temp);
			output.oneof_action_parameters.delay.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kExecuteAction:
		{
			decltype(output.oneof_action_parameters.execute_action)::value_type temp;
			ToRosData(input.execute_action(), temp);
			output.oneof_action_parameters.execute_action.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kSendGripperCommand:
		{
			decltype(output.oneof_action_parameters.send_gripper_command)::value_type temp;
			ToRosData(input.send_gripper_command(), temp);
			output.oneof_action_parameters.send_gripper_command.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kSendGpioCommand:
		{
			decltype(output.oneof_action_parameters.send_gpio_command)::value_type temp;
			ToRosData(input.send_gpio_command(), temp);
			output.oneof_action_parameters.send_gpio_command.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kStopAction:
		{
			decltype(output.oneof_action_parameters.stop_action)::value_type temp;
			ToRosData(input.stop_action(), temp);
			output.oneof_action_parameters.stop_action.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kPlayPreComputedTrajectory:
		{
			decltype(output.oneof_action_parameters.play_pre_computed_trajectory)::value_type temp;
			ToRosData(input.play_pre_computed_trajectory(), temp);
			output.oneof_action_parameters.play_pre_computed_trajectory.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kExecuteSequence:
		{
			decltype(output.oneof_action_parameters.execute_sequence)::value_type temp;
			ToRosData(input.execute_sequence(), temp);
			output.oneof_action_parameters.execute_sequence.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Action::kExecuteWaypointList:
		{
			decltype(output.oneof_action_parameters.execute_waypoint_list)::value_type temp;
			ToRosData(input.execute_waypoint_list(), temp);
			output.oneof_action_parameters.execute_waypoint_list.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Snapshot input, kortex_driver::Snapshot &output)
{
	
	output.snapshot_type = input.snapshot_type();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SwitchControlMapping input, kortex_driver::SwitchControlMapping &output)
{
	
	output.controller_identifier = input.controller_identifier();
	ToRosData(input.map_group_handle(), output.map_group_handle);
	ToRosData(input.map_handle(), output.map_handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ChangeTwist input, kortex_driver::ChangeTwist &output)
{
	
	output.linear = input.linear();
	output.angular = input.angular();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ChangeJointSpeeds input, kortex_driver::ChangeJointSpeeds &output)
{
	
	ToRosData(input.joint_speeds(), output.joint_speeds);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ChangeWrench input, kortex_driver::ChangeWrench &output)
{
	
	output.force = input.force();
	output.torque = input.torque();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::EmergencyStop input, kortex_driver::EmergencyStop &output)
{
	

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Faults input, kortex_driver::Faults &output)
{
	

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Delay input, kortex_driver::Delay &output)
{
	
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Stop input, kortex_driver::Base_Stop &output)
{
	

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActionList input, kortex_driver::ActionList &output)
{
	
	output.action_list.clear();
	for(int i = 0; i < input.action_list_size(); i++)
	{
		decltype(output.action_list)::value_type temp;
		ToRosData(input.action_list(i), temp);
		output.action_list.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Timeout input, kortex_driver::Timeout &output)
{
	
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Ssid input, kortex_driver::Ssid &output)
{
	
	output.identifier = input.identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CommunicationInterfaceConfiguration input, kortex_driver::CommunicationInterfaceConfiguration &output)
{
	
	output.type = input.type();
	output.enable = input.enable();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::NetworkHandle input, kortex_driver::NetworkHandle &output)
{
	
	output.type = input.type();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::IPv4Configuration input, kortex_driver::IPv4Configuration &output)
{
	
	output.ip_address = input.ip_address();
	output.subnet_mask = input.subnet_mask();
	output.default_gateway = input.default_gateway();
	output.dhcp_enabled = input.dhcp_enabled();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::IPv4Information input, kortex_driver::IPv4Information &output)
{
	
	output.ip_address = input.ip_address();
	output.subnet_mask = input.subnet_mask();
	output.default_gateway = input.default_gateway();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::FullIPv4Configuration input, kortex_driver::FullIPv4Configuration &output)
{
	
	ToRosData(input.handle(), output.handle);
	ToRosData(input.ipv4_configuration(), output.ipv4_configuration);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WifiInformation input, kortex_driver::WifiInformation &output)
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
int ToRosData(Kinova::Api::Base::WifiInformationList input, kortex_driver::WifiInformationList &output)
{
	
	output.wifi_information_list.clear();
	for(int i = 0; i < input.wifi_information_list_size(); i++)
	{
		decltype(output.wifi_information_list)::value_type temp;
		ToRosData(input.wifi_information_list(i), temp);
		output.wifi_information_list.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WifiConfiguration input, kortex_driver::WifiConfiguration &output)
{
	
	ToRosData(input.ssid(), output.ssid);
	output.security_key = input.security_key();
	output.connect_automatically = input.connect_automatically();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WifiConfigurationList input, kortex_driver::WifiConfigurationList &output)
{
	
	output.wifi_configuration_list.clear();
	for(int i = 0; i < input.wifi_configuration_list_size(); i++)
	{
		decltype(output.wifi_configuration_list)::value_type temp;
		ToRosData(input.wifi_configuration_list(i), temp);
		output.wifi_configuration_list.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ProtectionZoneHandle input, kortex_driver::ProtectionZoneHandle &output)
{
	
	output.identifier = input.identifier();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::RotationMatrixRow input, kortex_driver::Base_RotationMatrixRow &output)
{
	
	output.column1 = input.column1();
	output.column2 = input.column2();
	output.column3 = input.column3();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::RotationMatrix input, kortex_driver::Base_RotationMatrix &output)
{
	
	ToRosData(input.row1(), output.row1);
	ToRosData(input.row2(), output.row2);
	ToRosData(input.row3(), output.row3);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Point input, kortex_driver::Point &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ZoneShape input, kortex_driver::ZoneShape &output)
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
int ToRosData(Kinova::Api::Base::ProtectionZone input, kortex_driver::ProtectionZone &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.application_data = input.application_data();
	output.is_enabled = input.is_enabled();
	ToRosData(input.shape(), output.shape);
	output.limitations.clear();
	for(int i = 0; i < input.limitations_size(); i++)
	{
		decltype(output.limitations)::value_type temp;
		ToRosData(input.limitations(i), temp);
		output.limitations.push_back(temp);
	}
	output.envelope_limitations.clear();
	for(int i = 0; i < input.envelope_limitations_size(); i++)
	{
		decltype(output.envelope_limitations)::value_type temp;
		ToRosData(input.envelope_limitations(i), temp);
		output.envelope_limitations.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ProtectionZoneList input, kortex_driver::ProtectionZoneList &output)
{
	
	output.protection_zones.clear();
	for(int i = 0; i < input.protection_zones_size(); i++)
	{
		decltype(output.protection_zones)::value_type temp;
		ToRosData(input.protection_zones(i), temp);
		output.protection_zones.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CartesianLimitation input, kortex_driver::CartesianLimitation &output)
{
	
	output.type = input.type();
	output.translation = input.translation();
	output.orientation = input.orientation();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TwistLimitation input, kortex_driver::TwistLimitation &output)
{
	
	output.linear = input.linear();
	output.angular = input.angular();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WrenchLimitation input, kortex_driver::WrenchLimitation &output)
{
	
	output.force = input.force();
	output.torque = input.torque();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CartesianLimitationList input, kortex_driver::CartesianLimitationList &output)
{
	
	output.limitations.clear();
	for(int i = 0; i < input.limitations_size(); i++)
	{
		decltype(output.limitations)::value_type temp;
		ToRosData(input.limitations(i), temp);
		output.limitations.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointLimitation input, kortex_driver::JointLimitation &output)
{
	
	output.joint_identifier = input.joint_identifier();
	output.type = input.type();
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointsLimitationsList input, kortex_driver::JointsLimitationsList &output)
{
	
	output.joints_limitations.clear();
	for(int i = 0; i < input.joints_limitations_size(); i++)
	{
		decltype(output.joints_limitations)::value_type temp;
		ToRosData(input.joints_limitations(i), temp);
		output.joints_limitations.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Query input, kortex_driver::Query &output)
{
	
	ToRosData(input.start_timestamp(), output.start_timestamp);
	ToRosData(input.end_timestamp(), output.end_timestamp);
	output.username = input.username();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConfigurationChangeNotification input, kortex_driver::ConfigurationChangeNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	auto oneof_type_configuration_change = input.configuration_change_case();
	switch(oneof_type_configuration_change)
	{ 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kSequenceHandle:
		{
			decltype(output.oneof_configuration_change.sequence_handle)::value_type temp;
			ToRosData(input.sequence_handle(), temp);
			output.oneof_configuration_change.sequence_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kActionHandle:
		{
			decltype(output.oneof_configuration_change.action_handle)::value_type temp;
			ToRosData(input.action_handle(), temp);
			output.oneof_configuration_change.action_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kMappingHandle:
		{
			decltype(output.oneof_configuration_change.mapping_handle)::value_type temp;
			ToRosData(input.mapping_handle(), temp);
			output.oneof_configuration_change.mapping_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kMapGroupHandle:
		{
			decltype(output.oneof_configuration_change.map_group_handle)::value_type temp;
			ToRosData(input.map_group_handle(), temp);
			output.oneof_configuration_change.map_group_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kMapHandle:
		{
			decltype(output.oneof_configuration_change.map_handle)::value_type temp;
			ToRosData(input.map_handle(), temp);
			output.oneof_configuration_change.map_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kUserProfileHandle:
		{
			decltype(output.oneof_configuration_change.user_profile_handle)::value_type temp;
			ToRosData(input.user_profile_handle(), temp);
			output.oneof_configuration_change.user_profile_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kProtectionZoneHandle:
		{
			decltype(output.oneof_configuration_change.protection_zone_handle)::value_type temp;
			ToRosData(input.protection_zone_handle(), temp);
			output.oneof_configuration_change.protection_zone_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kSafetyHandle:
		{
			decltype(output.oneof_configuration_change.safety_handle)::value_type temp;
			ToRosData(input.safety_handle(), temp);
			output.oneof_configuration_change.safety_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kNetworkHandle:
		{
			decltype(output.oneof_configuration_change.network_handle)::value_type temp;
			ToRosData(input.network_handle(), temp);
			output.oneof_configuration_change.network_handle.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kSsid:
		{
			decltype(output.oneof_configuration_change.ssid)::value_type temp;
			ToRosData(input.ssid(), temp);
			output.oneof_configuration_change.ssid.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ConfigurationChangeNotification::kControllerHandle:
		{
			decltype(output.oneof_configuration_change.controller_handle)::value_type temp;
			ToRosData(input.controller_handle(), temp);
			output.oneof_configuration_change.controller_handle.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MappingInfoNotification input, kortex_driver::MappingInfoNotification &output)
{
	
	output.controller_identifier = input.controller_identifier();
	ToRosData(input.active_map_handle(), output.active_map_handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	ToRosData(input.mapping_handle(), output.mapping_handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControlModeInformation input, kortex_driver::Base_ControlModeInformation &output)
{
	
	output.mode = input.mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControlModeNotification input, kortex_driver::Base_ControlModeNotification &output)
{
	
	output.control_mode = input.control_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ServoingModeInformation input, kortex_driver::ServoingModeInformation &output)
{
	
	output.servoing_mode = input.servoing_mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::OperatingModeInformation input, kortex_driver::OperatingModeInformation &output)
{
	
	output.operating_mode = input.operating_mode();
	ToRosData(input.device_handle(), output.device_handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::OperatingModeNotification input, kortex_driver::OperatingModeNotification &output)
{
	
	output.operating_mode = input.operating_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);
	ToRosData(input.device_handle(), output.device_handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ServoingModeNotification input, kortex_driver::ServoingModeNotification &output)
{
	
	output.servoing_mode = input.servoing_mode();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceInfoNotification input, kortex_driver::SequenceInfoNotification &output)
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
int ToRosData(Kinova::Api::Base::SequenceInformation input, kortex_driver::SequenceInformation &output)
{
	
	output.event_identifier = input.event_identifier();
	output.task_index = input.task_index();
	output.task_identifier = input.task_identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ProtectionZoneNotification input, kortex_driver::ProtectionZoneNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.handle(), output.handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ProtectionZoneInformation input, kortex_driver::ProtectionZoneInformation &output)
{
	
	output.event = input.event();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::UserNotification input, kortex_driver::UserNotification &output)
{
	
	output.user_event = input.user_event();
	ToRosData(input.modified_user(), output.modified_user);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerHandle input, kortex_driver::ControllerHandle &output)
{
	
	output.type = input.type();
	output.controller_identifier = input.controller_identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerElementHandle input, kortex_driver::ControllerElementHandle &output)
{
	
	ToRosData(input.controller_handle(), output.controller_handle);

	
	auto oneof_type_identifier = input.identifier_case();
	switch(oneof_type_identifier)
	{ 
	
		case Kinova::Api::Base::ControllerElementHandle::kButton:
		{
			break;
		} 
	
		case Kinova::Api::Base::ControllerElementHandle::kAxis:
		{
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerNotification input, kortex_driver::ControllerNotification &output)
{
	
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	auto oneof_type_state = input.state_case();
	switch(oneof_type_state)
	{ 
	
		case Kinova::Api::Base::ControllerNotification::kControllerState:
		{
			decltype(output.oneof_state.controller_state)::value_type temp;
			ToRosData(input.controller_state(), temp);
			output.oneof_state.controller_state.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::ControllerNotification::kControllerElement:
		{
			decltype(output.oneof_state.controller_element)::value_type temp;
			ToRosData(input.controller_element(), temp);
			output.oneof_state.controller_element.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerList input, kortex_driver::ControllerList &output)
{
	
	output.handles.clear();
	for(int i = 0; i < input.handles_size(); i++)
	{
		decltype(output.handles)::value_type temp;
		ToRosData(input.handles(i), temp);
		output.handles.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerState input, kortex_driver::ControllerState &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.event_type = input.event_type();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerElementState input, kortex_driver::ControllerElementState &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.event_type = input.event_type();
	output.axis_value = input.axis_value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActionNotification input, kortex_driver::ActionNotification &output)
{
	
	output.action_event = input.action_event();
	ToRosData(input.handle(), output.handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	output.abort_details = input.abort_details();
	ToRosData(input.connection(), output.connection);
	output.trajectory_info.clear();
	for(int i = 0; i < input.trajectory_info_size(); i++)
	{
		decltype(output.trajectory_info)::value_type temp;
		ToRosData(input.trajectory_info(i), temp);
		output.trajectory_info.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TrajectoryInfo input, kortex_driver::TrajectoryInfo &output)
{
	
	output.trajectory_info_type = input.trajectory_info_type();
	output.waypoint_index = input.waypoint_index();
	output.joint_index = input.joint_index();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActionExecutionState input, kortex_driver::ActionExecutionState &output)
{
	
	output.action_event = input.action_event();
	ToRosData(input.handle(), output.handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::RobotEventNotification input, kortex_driver::RobotEventNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.handle(), output.handle);
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::FactoryNotification input, kortex_driver::FactoryNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::NetworkNotification input, kortex_driver::NetworkNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConfigurationChangeNotificationList input, kortex_driver::ConfigurationChangeNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MappingInfoNotificationList input, kortex_driver::MappingInfoNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControlModeNotificationList input, kortex_driver::ControlModeNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::OperatingModeNotificationList input, kortex_driver::OperatingModeNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ServoingModeNotificationList input, kortex_driver::ServoingModeNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SequenceInfoNotificationList input, kortex_driver::SequenceInfoNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ProtectionZoneNotificationList input, kortex_driver::ProtectionZoneNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::UserNotificationList input, kortex_driver::UserNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SafetyNotificationList input, kortex_driver::SafetyNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerNotificationList input, kortex_driver::ControllerNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActionNotificationList input, kortex_driver::ActionNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::RobotEventNotificationList input, kortex_driver::RobotEventNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::NetworkNotificationList input, kortex_driver::NetworkNotificationList &output)
{
	
	output.notifications.clear();
	for(int i = 0; i < input.notifications_size(); i++)
	{
		decltype(output.notifications)::value_type temp;
		ToRosData(input.notifications(i), temp);
		output.notifications.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MappingHandle input, kortex_driver::MappingHandle &output)
{
	
	output.identifier = input.identifier();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SafetyEvent input, kortex_driver::SafetyEvent &output)
{
	
	ToRosData(input.safety_handle(), output.safety_handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerEvent input, kortex_driver::ControllerEvent &output)
{
	
	output.input_type = input.input_type();
	output.behavior = input.behavior();
	output.input_identifier = input.input_identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::GpioEvent input, kortex_driver::GpioEvent &output)
{
	
	output.input_type = input.input_type();
	output.behavior = input.behavior();
	output.input_identifier = input.input_identifier();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapEvent input, kortex_driver::MapEvent &output)
{
	
	output.name = input.name();

	
	auto oneof_type_events = input.events_case();
	switch(oneof_type_events)
	{ 
	
		case Kinova::Api::Base::MapEvent::kSafetyEvent:
		{
			decltype(output.oneof_events.safety_event)::value_type temp;
			ToRosData(input.safety_event(), temp);
			output.oneof_events.safety_event.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::MapEvent::kGpioEvent:
		{
			decltype(output.oneof_events.gpio_event)::value_type temp;
			ToRosData(input.gpio_event(), temp);
			output.oneof_events.gpio_event.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::MapEvent::kControllerEvent:
		{
			decltype(output.oneof_events.controller_event)::value_type temp;
			ToRosData(input.controller_event(), temp);
			output.oneof_events.controller_event.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapElement input, kortex_driver::MapElement &output)
{
	
	ToRosData(input.event(), output.event);
	ToRosData(input.action(), output.action);
	output.name = input.name();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActivateMapHandle input, kortex_driver::ActivateMapHandle &output)
{
	
	ToRosData(input.mapping_handle(), output.mapping_handle);
	ToRosData(input.map_group_handle(), output.map_group_handle);
	ToRosData(input.map_handle(), output.map_handle);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Map input, kortex_driver::Map &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.elements.clear();
	for(int i = 0; i < input.elements_size(); i++)
	{
		decltype(output.elements)::value_type temp;
		ToRosData(input.elements(i), temp);
		output.elements.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapHandle input, kortex_driver::MapHandle &output)
{
	
	output.identifier = input.identifier();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapList input, kortex_driver::MapList &output)
{
	
	output.map_list.clear();
	for(int i = 0; i < input.map_list_size(); i++)
	{
		decltype(output.map_list)::value_type temp;
		ToRosData(input.map_list(i), temp);
		output.map_list.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapGroupHandle input, kortex_driver::MapGroupHandle &output)
{
	
	output.identifier = input.identifier();
	output.permission = input.permission();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapGroup input, kortex_driver::MapGroup &output)
{
	
	ToRosData(input.group_handle(), output.group_handle);
	output.name = input.name();
	ToRosData(input.related_mapping_handle(), output.related_mapping_handle);
	ToRosData(input.parent_group_handle(), output.parent_group_handle);
	output.children_map_group_handles.clear();
	for(int i = 0; i < input.children_map_group_handles_size(); i++)
	{
		decltype(output.children_map_group_handles)::value_type temp;
		ToRosData(input.children_map_group_handles(i), temp);
		output.children_map_group_handles.push_back(temp);
	}
	output.map_handles.clear();
	for(int i = 0; i < input.map_handles_size(); i++)
	{
		decltype(output.map_handles)::value_type temp;
		ToRosData(input.map_handles(i), temp);
		output.map_handles.push_back(temp);
	}
	output.application_data = input.application_data();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MapGroupList input, kortex_driver::MapGroupList &output)
{
	
	output.map_groups.clear();
	for(int i = 0; i < input.map_groups_size(); i++)
	{
		decltype(output.map_groups)::value_type temp;
		ToRosData(input.map_groups(i), temp);
		output.map_groups.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Mapping input, kortex_driver::Mapping &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	output.controller_identifier = input.controller_identifier();
	ToRosData(input.active_map_group_handle(), output.active_map_group_handle);
	output.map_group_handles.clear();
	for(int i = 0; i < input.map_group_handles_size(); i++)
	{
		decltype(output.map_group_handles)::value_type temp;
		ToRosData(input.map_group_handles(i), temp);
		output.map_group_handles.push_back(temp);
	}
	ToRosData(input.active_map_handle(), output.active_map_handle);
	output.map_handles.clear();
	for(int i = 0; i < input.map_handles_size(); i++)
	{
		decltype(output.map_handles)::value_type temp;
		ToRosData(input.map_handles(i), temp);
		output.map_handles.push_back(temp);
	}
	output.application_data = input.application_data();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::MappingList input, kortex_driver::MappingList &output)
{
	
	output.mappings.clear();
	for(int i = 0; i < input.mappings_size(); i++)
	{
		decltype(output.mappings)::value_type temp;
		ToRosData(input.mappings(i), temp);
		output.mappings.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TransformationMatrix input, kortex_driver::TransformationMatrix &output)
{
	
	ToRosData(input.r0(), output.r0);
	ToRosData(input.r1(), output.r1);
	ToRosData(input.r2(), output.r2);
	ToRosData(input.r3(), output.r3);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TransformationRow input, kortex_driver::TransformationRow &output)
{
	
	output.c0 = input.c0();
	output.c1 = input.c1();
	output.c2 = input.c2();
	output.c3 = input.c3();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Pose input, kortex_driver::Pose &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();
	output.theta_x = input.theta_x();
	output.theta_y = input.theta_y();
	output.theta_z = input.theta_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Position input, kortex_driver::Base_Position &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Orientation input, kortex_driver::Orientation &output)
{
	
	output.theta_x = input.theta_x();
	output.theta_y = input.theta_y();
	output.theta_z = input.theta_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CartesianSpeed input, kortex_driver::CartesianSpeed &output)
{
	
	output.translation = input.translation();
	output.orientation = input.orientation();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CartesianTrajectoryConstraint input, kortex_driver::CartesianTrajectoryConstraint &output)
{
	

	
	auto oneof_type_type = input.type_case();
	switch(oneof_type_type)
	{ 
	
		case Kinova::Api::Base::CartesianTrajectoryConstraint::kSpeed:
		{
			decltype(output.oneof_type.speed)::value_type temp;
			ToRosData(input.speed(), temp);
			output.oneof_type.speed.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::CartesianTrajectoryConstraint::kDuration:
		{
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointTrajectoryConstraint input, kortex_driver::JointTrajectoryConstraint &output)
{
	
	output.type = input.type();
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Wrench input, kortex_driver::Wrench &output)
{
	
	output.force_x = input.force_x();
	output.force_y = input.force_y();
	output.force_z = input.force_z();
	output.torque_x = input.torque_x();
	output.torque_y = input.torque_y();
	output.torque_z = input.torque_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Twist input, kortex_driver::Twist &output)
{
	
	output.linear_x = input.linear_x();
	output.linear_y = input.linear_y();
	output.linear_z = input.linear_z();
	output.angular_x = input.angular_x();
	output.angular_y = input.angular_y();
	output.angular_z = input.angular_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Admittance input, kortex_driver::Admittance &output)
{
	
	output.admittance_mode = input.admittance_mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConstrainedPose input, kortex_driver::ConstrainedPose &output)
{
	
	ToRosData(input.target_pose(), output.target_pose);
	ToRosData(input.constraint(), output.constraint);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConstrainedPosition input, kortex_driver::ConstrainedPosition &output)
{
	
	ToRosData(input.target_position(), output.target_position);
	ToRosData(input.constraint(), output.constraint);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConstrainedOrientation input, kortex_driver::ConstrainedOrientation &output)
{
	
	ToRosData(input.target_orientation(), output.target_orientation);
	ToRosData(input.constraint(), output.constraint);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WrenchCommand input, kortex_driver::WrenchCommand &output)
{
	
	output.reference_frame = input.reference_frame();
	output.mode = input.mode();
	ToRosData(input.wrench(), output.wrench);
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TwistCommand input, kortex_driver::TwistCommand &output)
{
	
	output.reference_frame = input.reference_frame();
	ToRosData(input.twist(), output.twist);
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConstrainedJointAngles input, kortex_driver::ConstrainedJointAngles &output)
{
	
	ToRosData(input.joint_angles(), output.joint_angles);
	ToRosData(input.constraint(), output.constraint);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ConstrainedJointAngle input, kortex_driver::ConstrainedJointAngle &output)
{
	
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();
	ToRosData(input.constraint(), output.constraint);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointAngles input, kortex_driver::JointAngles &output)
{
	
	output.joint_angles.clear();
	for(int i = 0; i < input.joint_angles_size(); i++)
	{
		decltype(output.joint_angles)::value_type temp;
		ToRosData(input.joint_angles(i), temp);
		output.joint_angles.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointAngle input, kortex_driver::JointAngle &output)
{
	
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointSpeeds input, kortex_driver::Base_JointSpeeds &output)
{
	
	output.joint_speeds.clear();
	for(int i = 0; i < input.joint_speeds_size(); i++)
	{
		decltype(output.joint_speeds)::value_type temp;
		ToRosData(input.joint_speeds(i), temp);
		output.joint_speeds.push_back(temp);
	}
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointSpeed input, kortex_driver::JointSpeed &output)
{
	
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointTorques input, kortex_driver::JointTorques &output)
{
	
	output.joint_torques.clear();
	for(int i = 0; i < input.joint_torques_size(); i++)
	{
		decltype(output.joint_torques)::value_type temp;
		ToRosData(input.joint_torques(i), temp);
		output.joint_torques.push_back(temp);
	}
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::JointTorque input, kortex_driver::JointTorque &output)
{
	
	output.joint_identifier = input.joint_identifier();
	output.value = input.value();
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::GripperCommand input, kortex_driver::GripperCommand &output)
{
	
	output.mode = input.mode();
	ToRosData(input.gripper(), output.gripper);
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::GripperRequest input, kortex_driver::GripperRequest &output)
{
	
	output.mode = input.mode();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Gripper input, kortex_driver::Gripper &output)
{
	
	output.finger.clear();
	for(int i = 0; i < input.finger_size(); i++)
	{
		decltype(output.finger)::value_type temp;
		ToRosData(input.finger(i), temp);
		output.finger.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Finger input, kortex_driver::Finger &output)
{
	
	output.finger_identifier = input.finger_identifier();
	output.value = input.value();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::GpioCommand input, kortex_driver::GpioCommand &output)
{
	
	output.port_identifier = input.port_identifier();
	output.pin_identifier = input.pin_identifier();
	output.action = input.action();
	output.period = input.period();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::SystemTime input, kortex_driver::SystemTime &output)
{
	
	output.sec = input.sec();
	output.min = input.min();
	output.hour = input.hour();
	output.mday = input.mday();
	output.mon = input.mon();
	output.year = input.year();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerConfigurationMode input, kortex_driver::ControllerConfigurationMode &output)
{
	
	output.enable = input.enable();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerConfiguration input, kortex_driver::ControllerConfiguration &output)
{
	
	ToRosData(input.handle(), output.handle);
	output.name = input.name();
	ToRosData(input.active_mapping_handle(), output.active_mapping_handle);
	output.analog_input_identifier_enum = input.analog_input_identifier_enum();
	output.digital_input_identifier_enum = input.digital_input_identifier_enum();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ControllerConfigurationList input, kortex_driver::ControllerConfigurationList &output)
{
	
	output.controller_configurations.clear();
	for(int i = 0; i < input.controller_configurations_size(); i++)
	{
		decltype(output.controller_configurations)::value_type temp;
		ToRosData(input.controller_configurations(i), temp);
		output.controller_configurations.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ActuatorInformation input, kortex_driver::ActuatorInformation &output)
{
	
	output.count = input.count();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ArmStateInformation input, kortex_driver::ArmStateInformation &output)
{
	
	output.active_state = input.active_state();
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::ArmStateNotification input, kortex_driver::ArmStateNotification &output)
{
	
	output.active_state = input.active_state();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CapSenseConfig input, kortex_driver::Base_CapSenseConfig &output)
{
	
	output.identifier = input.identifier();
	output.mode = input.mode();
	output.threshold_a = input.threshold_a();
	output.threshold_b = input.threshold_b();
	output.sensitivity_a = input.sensitivity_a();
	output.sensitivity_b = input.sensitivity_b();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::BridgeList input, kortex_driver::BridgeList &output)
{
	
	output.bridgeConfig.clear();
	for(int i = 0; i < input.bridgeconfig_size(); i++)
	{
		decltype(output.bridgeConfig)::value_type temp;
		ToRosData(input.bridgeconfig(i), temp);
		output.bridgeConfig.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::BridgeResult input, kortex_driver::BridgeResult &output)
{
	
	ToRosData(input.bridge_id(), output.bridge_id);
	output.status = input.status();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::BridgeIdentifier input, kortex_driver::BridgeIdentifier &output)
{
	
	output.bridge_id = input.bridge_id();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::BridgeConfig input, kortex_driver::BridgeConfig &output)
{
	
	output.device_identifier = input.device_identifier();
	output.bridgetype = input.bridgetype();
	ToRosData(input.port_config(), output.port_config);
	ToRosData(input.bridge_id(), output.bridge_id);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::BridgePortConfig input, kortex_driver::BridgePortConfig &output)
{
	
	output.target_port = input.target_port();
	output.out_port = input.out_port();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::PreComputedJointTrajectory input, kortex_driver::PreComputedJointTrajectory &output)
{
	
	output.mode = input.mode();
	output.trajectory_elements.clear();
	for(int i = 0; i < input.trajectory_elements_size(); i++)
	{
		decltype(output.trajectory_elements)::value_type temp;
		ToRosData(input.trajectory_elements(i), temp);
		output.trajectory_elements.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::PreComputedJointTrajectoryElement input, kortex_driver::PreComputedJointTrajectoryElement &output)
{
	
	output.joint_angles.clear();
	for(int i = 0; i < input.joint_angles_size(); i++)
	{
		output.joint_angles.push_back(input.joint_angles(i));
	}
	output.joint_speeds.clear();
	for(int i = 0; i < input.joint_speeds_size(); i++)
	{
		output.joint_speeds.push_back(input.joint_speeds(i));
	}
	output.joint_accelerations.clear();
	for(int i = 0; i < input.joint_accelerations_size(); i++)
	{
		output.joint_accelerations.push_back(input.joint_accelerations(i));
	}
	output.time_from_start = input.time_from_start();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TrajectoryErrorElement input, kortex_driver::TrajectoryErrorElement &output)
{
	
	output.error_type = input.error_type();
	output.error_identifier = input.error_identifier();
	output.error_value = input.error_value();
	output.min_value = input.min_value();
	output.max_value = input.max_value();
	output.index = input.index();
	output.message = input.message();
	output.waypoint_index = input.waypoint_index();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::TrajectoryErrorReport input, kortex_driver::TrajectoryErrorReport &output)
{
	
	output.trajectory_error_elements.clear();
	for(int i = 0; i < input.trajectory_error_elements_size(); i++)
	{
		decltype(output.trajectory_error_elements)::value_type temp;
		ToRosData(input.trajectory_error_elements(i), temp);
		output.trajectory_error_elements.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WaypointValidationReport input, kortex_driver::WaypointValidationReport &output)
{
	
	ToRosData(input.trajectory_error_report(), output.trajectory_error_report);
	ToRosData(input.optimal_waypoint_list(), output.optimal_waypoint_list);

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::Waypoint input, kortex_driver::Waypoint &output)
{
	
	output.name = input.name();

	
	auto oneof_type_type_of_waypoint = input.type_of_waypoint_case();
	switch(oneof_type_type_of_waypoint)
	{ 
	
		case Kinova::Api::Base::Waypoint::kAngularWaypoint:
		{
			decltype(output.oneof_type_of_waypoint.angular_waypoint)::value_type temp;
			ToRosData(input.angular_waypoint(), temp);
			output.oneof_type_of_waypoint.angular_waypoint.push_back(temp);
			break;
		} 
	
		case Kinova::Api::Base::Waypoint::kCartesianWaypoint:
		{
			decltype(output.oneof_type_of_waypoint.cartesian_waypoint)::value_type temp;
			ToRosData(input.cartesian_waypoint(), temp);
			output.oneof_type_of_waypoint.cartesian_waypoint.push_back(temp);
			break;
		}}
	
	return 0;
}
int ToRosData(Kinova::Api::Base::AngularWaypoint input, kortex_driver::AngularWaypoint &output)
{
	
	output.angles.clear();
	for(int i = 0; i < input.angles_size(); i++)
	{
		output.angles.push_back(input.angles(i));
	}
	output.maximum_velocities.clear();
	for(int i = 0; i < input.maximum_velocities_size(); i++)
	{
		output.maximum_velocities.push_back(input.maximum_velocities(i));
	}
	output.duration = input.duration();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::CartesianWaypoint input, kortex_driver::CartesianWaypoint &output)
{
	
	ToRosData(input.pose(), output.pose);
	output.reference_frame = input.reference_frame();
	output.maximum_linear_velocity = input.maximum_linear_velocity();
	output.maximum_angular_velocity = input.maximum_angular_velocity();
	output.blending_radius = input.blending_radius();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::WaypointList input, kortex_driver::WaypointList &output)
{
	
	output.waypoints.clear();
	for(int i = 0; i < input.waypoints_size(); i++)
	{
		decltype(output.waypoints)::value_type temp;
		ToRosData(input.waypoints(i), temp);
		output.waypoints.push_back(temp);
	}
	output.duration = input.duration();
	output.use_optimal_blending = input.use_optimal_blending();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::KinematicTrajectoryConstraints input, kortex_driver::KinematicTrajectoryConstraints &output)
{
	
	output.angular_velocities.clear();
	for(int i = 0; i < input.angular_velocities_size(); i++)
	{
		output.angular_velocities.push_back(input.angular_velocities(i));
	}
	output.linear_velocity = input.linear_velocity();
	output.angular_velocity = input.angular_velocity();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::FirmwareBundleVersions input, kortex_driver::FirmwareBundleVersions &output)
{
	
	output.main_bundle_version = input.main_bundle_version();
	output.components_versions.clear();
	for(int i = 0; i < input.components_versions_size(); i++)
	{
		decltype(output.components_versions)::value_type temp;
		ToRosData(input.components_versions(i), temp);
		output.components_versions.push_back(temp);
	}

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::FirmwareComponentVersion input, kortex_driver::FirmwareComponentVersion &output)
{
	
	output.name = input.name();
	output.version = input.version();
	output.device_id = input.device_id();

	
	
	return 0;
}
int ToRosData(Kinova::Api::Base::IKData input, kortex_driver::IKData &output)
{
	
	ToRosData(input.cartesian_pose(), output.cartesian_pose);
	ToRosData(input.guess(), output.guess);

	
	
	return 0;
}
