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
 
#include "kortex_driver/generated/common_proto_converter.h"
#include "kortex_driver/generated/common_ros_converter.h"
#include "kortex_driver/generated/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/base_proto_converter.h"
#include "kortex_driver/generated/base_ros_converter.h"
#include "kortex_driver/generated/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/basecyclic_proto_converter.h"
#include "kortex_driver/generated/basecyclic_ros_converter.h"
#include "kortex_driver/generated/controlconfig_proto_converter.h"
#include "kortex_driver/generated/controlconfig_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/devicemanager_proto_converter.h"
#include "kortex_driver/generated/devicemanager_ros_converter.h"
#include "kortex_driver/generated/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/visionconfig_proto_converter.h"
#include "kortex_driver/generated/visionconfig_ros_converter.h"
#include "kortex_driver/generated/base_services.h"

BaseServices::BaseServices(ros::NodeHandle& n, Kinova::Api::Base::BaseClient* base, uint32_t device_id, uint32_t timeout_ms): 
	m_n(n),
	m_base(base),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_n.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_ConfigurationChangeTopic = m_n.advertise<kortex_driver::ConfigurationChangeNotification>("configuration_change_topic", 1000);
	m_pub_MappingInfoTopic = m_n.advertise<kortex_driver::MappingInfoNotification>("mapping_info_topic", 1000);
	m_pub_ControlModeTopic = m_n.advertise<kortex_driver::ControlModeNotification>("control_mode_topic", 1000);
	m_pub_OperatingModeTopic = m_n.advertise<kortex_driver::OperatingModeNotification>("operating_mode_topic", 1000);
	m_pub_SequenceInfoTopic = m_n.advertise<kortex_driver::SequenceInfoNotification>("sequence_info_topic", 1000);
	m_pub_ProtectionZoneTopic = m_n.advertise<kortex_driver::ProtectionZoneNotification>("protection_zone_topic", 1000);
	m_pub_UserTopic = m_n.advertise<kortex_driver::UserNotification>("user_topic", 1000);
	m_pub_ControllerTopic = m_n.advertise<kortex_driver::ControllerNotification>("controller_topic", 1000);
	m_pub_ActionTopic = m_n.advertise<kortex_driver::ActionNotification>("action_topic", 1000);
	m_pub_RobotEventTopic = m_n.advertise<kortex_driver::RobotEventNotification>("robot_event_topic", 1000);
	m_pub_ServoingModeTopic = m_n.advertise<kortex_driver::ServoingModeNotification>("servoing_mode_topic", 1000);
	m_pub_FactoryTopic = m_n.advertise<kortex_driver::FactoryNotification>("factory_topic", 1000);
	m_pub_NetworkTopic = m_n.advertise<kortex_driver::NetworkNotification>("network_topic", 1000);
	m_pub_ArmStateTopic = m_n.advertise<kortex_driver::ArmStateNotification>("arm_state_topic", 1000);

	m_serviceSetDeviceID = n.advertiseService("base/set_device_id", &BaseServices::SetDeviceID, this);
	m_serviceSetApiOptions = n.advertiseService("base/set_api_options", &BaseServices::SetApiOptions, this);

	m_serviceCreateUserProfile = m_n.advertiseService("base/create_user_profile", &BaseServices::CreateUserProfile, this);
	m_serviceUpdateUserProfile = m_n.advertiseService("base/update_user_profile", &BaseServices::UpdateUserProfile, this);
	m_serviceReadUserProfile = m_n.advertiseService("base/read_user_profile", &BaseServices::ReadUserProfile, this);
	m_serviceDeleteUserProfile = m_n.advertiseService("base/delete_user_profile", &BaseServices::DeleteUserProfile, this);
	m_serviceReadAllUserProfiles = m_n.advertiseService("base/read_all_user_profiles", &BaseServices::ReadAllUserProfiles, this);
	m_serviceReadAllUsers = m_n.advertiseService("base/read_all_users", &BaseServices::ReadAllUsers, this);
	m_serviceChangePassword = m_n.advertiseService("base/change_password", &BaseServices::ChangePassword, this);
	m_serviceCreateSequence = m_n.advertiseService("base/create_sequence", &BaseServices::CreateSequence, this);
	m_serviceUpdateSequence = m_n.advertiseService("base/update_sequence", &BaseServices::UpdateSequence, this);
	m_serviceReadSequence = m_n.advertiseService("base/read_sequence", &BaseServices::ReadSequence, this);
	m_serviceDeleteSequence = m_n.advertiseService("base/delete_sequence", &BaseServices::DeleteSequence, this);
	m_serviceReadAllSequences = m_n.advertiseService("base/read_all_sequences", &BaseServices::ReadAllSequences, this);
	m_serviceDeleteSequenceTask = m_n.advertiseService("base/delete_sequence_task", &BaseServices::DeleteSequenceTask, this);
	m_serviceDeleteAllSequenceTasks = m_n.advertiseService("base/delete_all_sequence_tasks", &BaseServices::DeleteAllSequenceTasks, this);
	m_servicePlaySequence = m_n.advertiseService("base/play_sequence", &BaseServices::PlaySequence, this);
	m_servicePlayAdvancedSequence = m_n.advertiseService("base/play_advanced_sequence", &BaseServices::PlayAdvancedSequence, this);
	m_serviceStopSequence = m_n.advertiseService("base/stop_sequence", &BaseServices::StopSequence, this);
	m_servicePauseSequence = m_n.advertiseService("base/pause_sequence", &BaseServices::PauseSequence, this);
	m_serviceResumeSequence = m_n.advertiseService("base/resume_sequence", &BaseServices::ResumeSequence, this);
	m_serviceCreateProtectionZone = m_n.advertiseService("base/create_protection_zone", &BaseServices::CreateProtectionZone, this);
	m_serviceUpdateProtectionZone = m_n.advertiseService("base/update_protection_zone", &BaseServices::UpdateProtectionZone, this);
	m_serviceReadProtectionZone = m_n.advertiseService("base/read_protection_zone", &BaseServices::ReadProtectionZone, this);
	m_serviceDeleteProtectionZone = m_n.advertiseService("base/delete_protection_zone", &BaseServices::DeleteProtectionZone, this);
	m_serviceReadAllProtectionZones = m_n.advertiseService("base/read_all_protection_zones", &BaseServices::ReadAllProtectionZones, this);
	m_serviceCreateMapping = m_n.advertiseService("base/create_mapping", &BaseServices::CreateMapping, this);
	m_serviceReadMapping = m_n.advertiseService("base/read_mapping", &BaseServices::ReadMapping, this);
	m_serviceReadAllMappings = m_n.advertiseService("base/read_all_mappings", &BaseServices::ReadAllMappings, this);
	m_serviceCreateMap = m_n.advertiseService("base/create_map", &BaseServices::CreateMap, this);
	m_serviceReadAllMaps = m_n.advertiseService("base/read_all_maps", &BaseServices::ReadAllMaps, this);
	m_serviceActivateMap = m_n.advertiseService("base/activate_map", &BaseServices::ActivateMap, this);
	m_serviceCreateAction = m_n.advertiseService("base/create_action", &BaseServices::CreateAction, this);
	m_serviceReadAction = m_n.advertiseService("base/read_action", &BaseServices::ReadAction, this);
	m_serviceReadAllActions = m_n.advertiseService("base/read_all_actions", &BaseServices::ReadAllActions, this);
	m_serviceDeleteAction = m_n.advertiseService("base/delete_action", &BaseServices::DeleteAction, this);
	m_serviceUpdateAction = m_n.advertiseService("base/update_action", &BaseServices::UpdateAction, this);
	m_serviceExecuteActionFromReference = m_n.advertiseService("base/execute_action_from_reference", &BaseServices::ExecuteActionFromReference, this);
	m_serviceExecuteAction = m_n.advertiseService("base/execute_action", &BaseServices::ExecuteAction, this);
	m_servicePauseAction = m_n.advertiseService("base/pause_action", &BaseServices::PauseAction, this);
	m_serviceStopAction = m_n.advertiseService("base/stop_action", &BaseServices::StopAction, this);
	m_serviceResumeAction = m_n.advertiseService("base/resume_action", &BaseServices::ResumeAction, this);
	m_serviceGetIPv4Configuration = m_n.advertiseService("base/get_i_pv4_configuration", &BaseServices::GetIPv4Configuration, this);
	m_serviceSetIPv4Configuration = m_n.advertiseService("base/set_i_pv4_configuration", &BaseServices::SetIPv4Configuration, this);
	m_serviceSetCommunicationInterfaceEnable = m_n.advertiseService("base/set_communication_interface_enable", &BaseServices::SetCommunicationInterfaceEnable, this);
	m_serviceIsCommunicationInterfaceEnable = m_n.advertiseService("base/is_communication_interface_enable", &BaseServices::IsCommunicationInterfaceEnable, this);
	m_serviceGetAvailableWifi = m_n.advertiseService("base/get_available_wifi", &BaseServices::GetAvailableWifi, this);
	m_serviceGetWifiInformation = m_n.advertiseService("base/get_wifi_information", &BaseServices::GetWifiInformation, this);
	m_serviceAddWifiConfiguration = m_n.advertiseService("base/add_wifi_configuration", &BaseServices::AddWifiConfiguration, this);
	m_serviceDeleteWifiConfiguration = m_n.advertiseService("base/delete_wifi_configuration", &BaseServices::DeleteWifiConfiguration, this);
	m_serviceGetAllConfiguredWifis = m_n.advertiseService("base/get_all_configured_wifis", &BaseServices::GetAllConfiguredWifis, this);
	m_serviceConnectWifi = m_n.advertiseService("base/connect_wifi", &BaseServices::ConnectWifi, this);
	m_serviceDisconnectWifi = m_n.advertiseService("base/disconnect_wifi", &BaseServices::DisconnectWifi, this);
	m_serviceGetConnectedWifiInformation = m_n.advertiseService("base/get_connected_wifi_information", &BaseServices::GetConnectedWifiInformation, this);
	m_serviceBase_Unsubscribe = m_n.advertiseService("base/unsubscribe", &BaseServices::Base_Unsubscribe, this);
	m_serviceOnNotificationConfigurationChangeTopic = m_n.advertiseService("base/activate_publishing_of_configuration_change_topic", &BaseServices::OnNotificationConfigurationChangeTopic, this);
	m_serviceOnNotificationMappingInfoTopic = m_n.advertiseService("base/activate_publishing_of_mapping_info_topic", &BaseServices::OnNotificationMappingInfoTopic, this);
	m_serviceOnNotificationControlModeTopic = m_n.advertiseService("base/activate_publishing_of_control_mode_topic", &BaseServices::OnNotificationControlModeTopic, this);
	m_serviceOnNotificationOperatingModeTopic = m_n.advertiseService("base/activate_publishing_of_operating_mode_topic", &BaseServices::OnNotificationOperatingModeTopic, this);
	m_serviceOnNotificationSequenceInfoTopic = m_n.advertiseService("base/activate_publishing_of_sequence_info_topic", &BaseServices::OnNotificationSequenceInfoTopic, this);
	m_serviceOnNotificationProtectionZoneTopic = m_n.advertiseService("base/activate_publishing_of_protection_zone_topic", &BaseServices::OnNotificationProtectionZoneTopic, this);
	m_serviceOnNotificationUserTopic = m_n.advertiseService("base/activate_publishing_of_user_topic", &BaseServices::OnNotificationUserTopic, this);
	m_serviceOnNotificationControllerTopic = m_n.advertiseService("base/activate_publishing_of_controller_topic", &BaseServices::OnNotificationControllerTopic, this);
	m_serviceOnNotificationActionTopic = m_n.advertiseService("base/activate_publishing_of_action_topic", &BaseServices::OnNotificationActionTopic, this);
	m_serviceOnNotificationRobotEventTopic = m_n.advertiseService("base/activate_publishing_of_robot_event_topic", &BaseServices::OnNotificationRobotEventTopic, this);
	m_servicePlayCartesianTrajectory = m_n.advertiseService("base/play_cartesian_trajectory", &BaseServices::PlayCartesianTrajectory, this);
	m_servicePlayCartesianTrajectoryPosition = m_n.advertiseService("base/play_cartesian_trajectory_position", &BaseServices::PlayCartesianTrajectoryPosition, this);
	m_servicePlayCartesianTrajectoryOrientation = m_n.advertiseService("base/play_cartesian_trajectory_orientation", &BaseServices::PlayCartesianTrajectoryOrientation, this);
	m_serviceStop = m_n.advertiseService("base/stop", &BaseServices::Stop, this);
	m_serviceGetMeasuredCartesianPose = m_n.advertiseService("base/get_measured_cartesian_pose", &BaseServices::GetMeasuredCartesianPose, this);
	m_serviceSendWrenchCommand = m_n.advertiseService("base/send_wrench_command", &BaseServices::SendWrenchCommand, this);
	m_serviceSendWrenchJoystickCommand = m_n.advertiseService("base/send_wrench_joystick_command", &BaseServices::SendWrenchJoystickCommand, this);
	m_serviceSendTwistJoystickCommand = m_n.advertiseService("base/send_twist_joystick_command", &BaseServices::SendTwistJoystickCommand, this);
	m_serviceSendTwistCommand = m_n.advertiseService("base/send_twist_command", &BaseServices::SendTwistCommand, this);
	m_servicePlayJointTrajectory = m_n.advertiseService("base/play_joint_trajectory", &BaseServices::PlayJointTrajectory, this);
	m_servicePlaySelectedJointTrajectory = m_n.advertiseService("base/play_selected_joint_trajectory", &BaseServices::PlaySelectedJointTrajectory, this);
	m_serviceGetMeasuredJointAngles = m_n.advertiseService("base/get_measured_joint_angles", &BaseServices::GetMeasuredJointAngles, this);
	m_serviceSendJointSpeedsCommand = m_n.advertiseService("base/send_joint_speeds_command", &BaseServices::SendJointSpeedsCommand, this);
	m_serviceSendSelectedJointSpeedCommand = m_n.advertiseService("base/send_selected_joint_speed_command", &BaseServices::SendSelectedJointSpeedCommand, this);
	m_serviceSendGripperCommand = m_n.advertiseService("base/send_gripper_command", &BaseServices::SendGripperCommand, this);
	m_serviceGetMeasuredGripperMovement = m_n.advertiseService("base/get_measured_gripper_movement", &BaseServices::GetMeasuredGripperMovement, this);
	m_serviceSetAdmittance = m_n.advertiseService("base/set_admittance", &BaseServices::SetAdmittance, this);
	m_serviceSetOperatingMode = m_n.advertiseService("base/set_operating_mode", &BaseServices::SetOperatingMode, this);
	m_serviceApplyEmergencyStop = m_n.advertiseService("base/apply_emergency_stop", &BaseServices::ApplyEmergencyStop, this);
	m_serviceBase_ClearFaults = m_n.advertiseService("base/clear_faults", &BaseServices::Base_ClearFaults, this);
	m_serviceBase_GetControlMode = m_n.advertiseService("base/get_control_mode", &BaseServices::Base_GetControlMode, this);
	m_serviceGetOperatingMode = m_n.advertiseService("base/get_operating_mode", &BaseServices::GetOperatingMode, this);
	m_serviceSetServoingMode = m_n.advertiseService("base/set_servoing_mode", &BaseServices::SetServoingMode, this);
	m_serviceGetServoingMode = m_n.advertiseService("base/get_servoing_mode", &BaseServices::GetServoingMode, this);
	m_serviceOnNotificationServoingModeTopic = m_n.advertiseService("base/activate_publishing_of_servoing_mode_topic", &BaseServices::OnNotificationServoingModeTopic, this);
	m_serviceRestoreFactorySettings = m_n.advertiseService("base/restore_factory_settings", &BaseServices::RestoreFactorySettings, this);
	m_serviceOnNotificationFactoryTopic = m_n.advertiseService("base/activate_publishing_of_factory_topic", &BaseServices::OnNotificationFactoryTopic, this);
	m_serviceGetAllConnectedControllers = m_n.advertiseService("base/get_all_connected_controllers", &BaseServices::GetAllConnectedControllers, this);
	m_serviceGetControllerState = m_n.advertiseService("base/get_controller_state", &BaseServices::GetControllerState, this);
	m_serviceGetActuatorCount = m_n.advertiseService("base/get_actuator_count", &BaseServices::GetActuatorCount, this);
	m_serviceStartWifiScan = m_n.advertiseService("base/start_wifi_scan", &BaseServices::StartWifiScan, this);
	m_serviceGetConfiguredWifi = m_n.advertiseService("base/get_configured_wifi", &BaseServices::GetConfiguredWifi, this);
	m_serviceOnNotificationNetworkTopic = m_n.advertiseService("base/activate_publishing_of_network_topic", &BaseServices::OnNotificationNetworkTopic, this);
	m_serviceGetArmState = m_n.advertiseService("base/get_arm_state", &BaseServices::GetArmState, this);
	m_serviceOnNotificationArmStateTopic = m_n.advertiseService("base/activate_publishing_of_arm_state_topic", &BaseServices::OnNotificationArmStateTopic, this);
	m_serviceGetIPv4Information = m_n.advertiseService("base/get_i_pv4_information", &BaseServices::GetIPv4Information, this);
	m_serviceSetWifiCountryCode = m_n.advertiseService("base/set_wifi_country_code", &BaseServices::SetWifiCountryCode, this);
	m_serviceGetWifiCountryCode = m_n.advertiseService("base/get_wifi_country_code", &BaseServices::GetWifiCountryCode, this);
	m_serviceBase_SetCapSenseConfig = m_n.advertiseService("base/set_cap_sense_config", &BaseServices::Base_SetCapSenseConfig, this);
	m_serviceBase_GetCapSenseConfig = m_n.advertiseService("base/get_cap_sense_config", &BaseServices::Base_GetCapSenseConfig, this);
	m_serviceGetAllJointsSpeedHardLimitation = m_n.advertiseService("base/get_all_joints_speed_hard_limitation", &BaseServices::GetAllJointsSpeedHardLimitation, this);
	m_serviceGetAllJointsTorqueHardLimitation = m_n.advertiseService("base/get_all_joints_torque_hard_limitation", &BaseServices::GetAllJointsTorqueHardLimitation, this);
	m_serviceGetTwistHardLimitation = m_n.advertiseService("base/get_twist_hard_limitation", &BaseServices::GetTwistHardLimitation, this);
	m_serviceGetWrenchHardLimitation = m_n.advertiseService("base/get_wrench_hard_limitation", &BaseServices::GetWrenchHardLimitation, this);
	m_serviceSendJointSpeedsJoystickCommand = m_n.advertiseService("base/send_joint_speeds_joystick_command", &BaseServices::SendJointSpeedsJoystickCommand, this);
	m_serviceSendSelectedJointSpeedJoystickCommand = m_n.advertiseService("base/send_selected_joint_speed_joystick_command", &BaseServices::SendSelectedJointSpeedJoystickCommand, this);
	m_serviceEnableBridge = m_n.advertiseService("base/enable_bridge", &BaseServices::EnableBridge, this);
	m_serviceDisableBridge = m_n.advertiseService("base/disable_bridge", &BaseServices::DisableBridge, this);
	m_serviceGetBridgeList = m_n.advertiseService("base/get_bridge_list", &BaseServices::GetBridgeList, this);
	m_serviceGetBridgeConfig = m_n.advertiseService("base/get_bridge_config", &BaseServices::GetBridgeConfig, this);
	m_servicePlayPreComputedJointTrajectory = m_n.advertiseService("base/play_pre_computed_joint_trajectory", &BaseServices::PlayPreComputedJointTrajectory, this);
	m_serviceGetProductConfiguration = m_n.advertiseService("base/get_product_configuration", &BaseServices::GetProductConfiguration, this);
	m_serviceUpdateDegreeOfFreedomConfiguration = m_n.advertiseService("base/update_degree_of_freedom_configuration", &BaseServices::UpdateDegreeOfFreedomConfiguration, this);
	m_serviceUpdateBaseTypeConfiguration = m_n.advertiseService("base/update_base_type_configuration", &BaseServices::UpdateBaseTypeConfiguration, this);
	m_serviceUpdateEndEffectorTypeConfiguration = m_n.advertiseService("base/update_end_effector_type_configuration", &BaseServices::UpdateEndEffectorTypeConfiguration, this);
	m_serviceUpdateVisionModuleTypeConfiguration = m_n.advertiseService("base/update_vision_module_type_configuration", &BaseServices::UpdateVisionModuleTypeConfiguration, this);
	m_serviceUpdateInterfaceModuleTypeConfiguration = m_n.advertiseService("base/update_interface_module_type_configuration", &BaseServices::UpdateInterfaceModuleTypeConfiguration, this);
	m_serviceUpdateArmLateralityConfiguration = m_n.advertiseService("base/update_arm_laterality_configuration", &BaseServices::UpdateArmLateralityConfiguration, this);
	m_serviceUpdateWristTypeConfiguration = m_n.advertiseService("base/update_wrist_type_configuration", &BaseServices::UpdateWristTypeConfiguration, this);
	m_serviceRestoreFactoryProductConfiguration = m_n.advertiseService("base/restore_factory_product_configuration", &BaseServices::RestoreFactoryProductConfiguration, this);
	m_serviceGetTrajectoryErrorReport = m_n.advertiseService("base/get_trajectory_error_report", &BaseServices::GetTrajectoryErrorReport, this);
	m_serviceGetAllJointsSpeedSoftLimitation = m_n.advertiseService("base/get_all_joints_speed_soft_limitation", &BaseServices::GetAllJointsSpeedSoftLimitation, this);
	m_serviceGetAllJointsTorqueSoftLimitation = m_n.advertiseService("base/get_all_joints_torque_soft_limitation", &BaseServices::GetAllJointsTorqueSoftLimitation, this);
	m_serviceGetTwistSoftLimitation = m_n.advertiseService("base/get_twist_soft_limitation", &BaseServices::GetTwistSoftLimitation, this);
	m_serviceGetWrenchSoftLimitation = m_n.advertiseService("base/get_wrench_soft_limitation", &BaseServices::GetWrenchSoftLimitation, this);
}

bool BaseServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool BaseServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool BaseServices::CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res)
{
	Kinova::Api::Base::FullUserProfile input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::UserProfileHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateUserProfile(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res)
{
	Kinova::Api::Base::UserProfile input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateUserProfile(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res)
{
	Kinova::Api::Common::UserProfileHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::UserProfile output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadUserProfile(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res)
{
	Kinova::Api::Common::UserProfileHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteUserProfile(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res)
{
	Kinova::Api::Base::UserProfileList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllUserProfiles(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res)
{
	Kinova::Api::Base::UserList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllUsers(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res)
{
	Kinova::Api::Base::PasswordChange input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ChangePassword(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res)
{
	Kinova::Api::Base::Sequence input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::SequenceHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateSequence(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res)
{
	Kinova::Api::Base::Sequence input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateSequence(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res)
{
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::Sequence output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadSequence(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res)
{
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteSequence(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res)
{
	Kinova::Api::Base::SequenceList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllSequences(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res)
{
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteSequenceTask(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res)
{
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteAllSequenceTasks(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res)
{
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlaySequence(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res)
{
	Kinova::Api::Base::AdvancedSequenceHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayAdvancedSequence(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StopSequence(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PauseSequence(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ResumeSequence(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res)
{
	Kinova::Api::Base::ProtectionZone input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::ProtectionZoneHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateProtectionZone(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res)
{
	Kinova::Api::Base::ProtectionZone input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateProtectionZone(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res)
{
	Kinova::Api::Base::ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::ProtectionZone output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadProtectionZone(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res)
{
	Kinova::Api::Base::ProtectionZoneHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteProtectionZone(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res)
{
	Kinova::Api::Base::ProtectionZoneList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllProtectionZones(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res)
{
	Kinova::Api::Base::Mapping input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::MappingHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateMapping(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res)
{
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::Mapping output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadMapping(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res)
{
	Kinova::Api::Base::MappingList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllMappings(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res)
{
	Kinova::Api::Base::Map input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::MapHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateMap(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res)
{
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::MapList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllMaps(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res)
{
	Kinova::Api::Base::ActivateMapHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ActivateMap(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res)
{
	Kinova::Api::Base::Action input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::ActionHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->CreateAction(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res)
{
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::Action output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAction(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res)
{
	Kinova::Api::Base::RequestedActionType input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::ActionList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllActions(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res)
{
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteAction(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res)
{
	Kinova::Api::Base::Action input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateAction(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res)
{
	Kinova::Api::Base::ActionHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ExecuteActionFromReference(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res)
{
	Kinova::Api::Base::Action input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ExecuteAction(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PauseAction(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StopAction(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ResumeAction(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res)
{
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::IPv4Configuration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetIPv4Configuration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res)
{
	Kinova::Api::Base::FullIPv4Configuration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetIPv4Configuration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res)
{
	Kinova::Api::Base::CommunicationInterfaceConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetCommunicationInterfaceEnable(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res)
{
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::CommunicationInterfaceConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->IsCommunicationInterfaceEnable(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res)
{
	Kinova::Api::Base::WifiInformationList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAvailableWifi(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res)
{
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::WifiInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetWifiInformation(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res)
{
	Kinova::Api::Base::WifiConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->AddWifiConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res)
{
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteWifiConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res)
{
	Kinova::Api::Base::WifiConfigurationList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllConfiguredWifis(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res)
{
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ConnectWifi(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DisconnectWifi(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res)
{
	Kinova::Api::Base::WifiInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetConnectedWifiInformation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request  &req, kortex_driver::Base_Unsubscribe::Response &res)
{
	Kinova::Api::Common::NotificationHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->Unsubscribe(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ConfigurationChangeNotification) > callback = std::bind(&BaseServices::cb_ConfigurationChangeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationConfigurationChangeTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif)
{
	kortex_driver::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::MappingInfoNotification) > callback = std::bind(&BaseServices::cb_MappingInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationMappingInfoTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif)
{
	kortex_driver::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationControlModeTopic(kortex_driver::OnNotificationControlModeTopic::Request  &req, kortex_driver::OnNotificationControlModeTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ControlModeNotification) > callback = std::bind(&BaseServices::cb_ControlModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControlModeTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif)
{
	kortex_driver::ControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::OperatingModeNotification) > callback = std::bind(&BaseServices::cb_OperatingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationOperatingModeTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif)
{
	kortex_driver::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::SequenceInfoNotification) > callback = std::bind(&BaseServices::cb_SequenceInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationSequenceInfoTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif)
{
	kortex_driver::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ProtectionZoneNotification) > callback = std::bind(&BaseServices::cb_ProtectionZoneTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationProtectionZoneTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif)
{
	kortex_driver::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::UserNotification) > callback = std::bind(&BaseServices::cb_UserTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationUserTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_UserTopic(Kinova::Api::Base::UserNotification notif)
{
	kortex_driver::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ControllerNotification) > callback = std::bind(&BaseServices::cb_ControllerTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControllerTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif)
{
	kortex_driver::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ActionNotification) > callback = std::bind(&BaseServices::cb_ActionTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationActionTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ActionTopic(Kinova::Api::Base::ActionNotification notif)
{
	kortex_driver::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic.publish(ros_msg);
}

bool BaseServices::OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::RobotEventNotification) > callback = std::bind(&BaseServices::cb_RobotEventTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationRobotEventTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif)
{
	kortex_driver::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic.publish(ros_msg);
}

bool BaseServices::PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res)
{
	Kinova::Api::Base::ConstrainedPose input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectory(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res)
{
	Kinova::Api::Base::ConstrainedPosition input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectoryPosition(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res)
{
	Kinova::Api::Base::ConstrainedOrientation input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayCartesianTrajectoryOrientation(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::Stop(kortex_driver::Stop::Request  &req, kortex_driver::Stop::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->Stop(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res)
{
	Kinova::Api::Base::Pose output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredCartesianPose(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SendWrenchCommand(kortex_driver::SendWrenchCommand::Request  &req, kortex_driver::SendWrenchCommand::Response &res)
{
	Kinova::Api::Base::WrenchCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendWrenchCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request  &req, kortex_driver::SendWrenchJoystickCommand::Response &res)
{
	Kinova::Api::Base::WrenchCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendWrenchJoystickCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request  &req, kortex_driver::SendTwistJoystickCommand::Response &res)
{
	Kinova::Api::Base::TwistCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendTwistJoystickCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res)
{
	Kinova::Api::Base::TwistCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendTwistCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res)
{
	Kinova::Api::Base::ConstrainedJointAngles input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayJointTrajectory(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res)
{
	Kinova::Api::Base::ConstrainedJointAngle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlaySelectedJointTrajectory(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res)
{
	Kinova::Api::Base::JointAngles output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredJointAngles(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request  &req, kortex_driver::SendJointSpeedsCommand::Response &res)
{
	Kinova::Api::Base::JointSpeeds input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendJointSpeedsCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res)
{
	Kinova::Api::Base::JointSpeed input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendSelectedJointSpeedCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res)
{
	Kinova::Api::Base::GripperCommand input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendGripperCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res)
{
	Kinova::Api::Base::GripperRequest input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::Gripper output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetMeasuredGripperMovement(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res)
{
	Kinova::Api::Base::Admittance input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetAdmittance(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res)
{
	Kinova::Api::Base::OperatingModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetOperatingMode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ApplyEmergencyStop(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::Base_ClearFaults(kortex_driver::Base_ClearFaults::Request  &req, kortex_driver::Base_ClearFaults::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ClearFaults(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::Base_GetControlMode(kortex_driver::Base_GetControlMode::Request  &req, kortex_driver::Base_GetControlMode::Response &res)
{
	Kinova::Api::Base::ControlModeInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetControlMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res)
{
	Kinova::Api::Base::OperatingModeInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetOperatingMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res)
{
	Kinova::Api::Base::ServoingModeInformation input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetServoingMode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res)
{
	Kinova::Api::Base::ServoingModeInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetServoingMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ServoingModeNotification) > callback = std::bind(&BaseServices::cb_ServoingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationServoingModeTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif)
{
	kortex_driver::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic.publish(ros_msg);
}

bool BaseServices::RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->RestoreFactorySettings(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::FactoryNotification) > callback = std::bind(&BaseServices::cb_FactoryTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationFactoryTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif)
{
	kortex_driver::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic.publish(ros_msg);
}

bool BaseServices::GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res)
{
	Kinova::Api::Base::ControllerList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllConnectedControllers(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res)
{
	Kinova::Api::Base::ControllerHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::ControllerState output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerState(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res)
{
	Kinova::Api::Base::ActuatorInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetActuatorCount(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StartWifiScan(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res)
{
	Kinova::Api::Base::Ssid input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::WifiConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetConfiguredWifi(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::NetworkNotification) > callback = std::bind(&BaseServices::cb_NetworkTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationNetworkTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif)
{
	kortex_driver::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic.publish(ros_msg);
}

bool BaseServices::GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res)
{
	Kinova::Api::Base::ArmStateInformation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetArmState(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res)
{
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ArmStateNotification) > callback = std::bind(&BaseServices::cb_ArmStateTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationArmStateTopic(callback, input, m_current_device_id);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
void BaseServices::cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif)
{
	kortex_driver::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic.publish(ros_msg);
}

bool BaseServices::GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res)
{
	Kinova::Api::Base::NetworkHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::IPv4Information output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetIPv4Information(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request  &req, kortex_driver::SetWifiCountryCode::Response &res)
{
	Kinova::Api::Common::CountryCode input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetWifiCountryCode(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request  &req, kortex_driver::GetWifiCountryCode::Response &res)
{
	Kinova::Api::Common::CountryCode output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetWifiCountryCode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request  &req, kortex_driver::Base_SetCapSenseConfig::Response &res)
{
	Kinova::Api::Base::CapSenseConfig input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetCapSenseConfig(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request  &req, kortex_driver::Base_GetCapSenseConfig::Response &res)
{
	Kinova::Api::Base::CapSenseConfig output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetCapSenseConfig(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::GetAllJointsSpeedHardLimitation::Response &res)
{
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsSpeedHardLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::GetAllJointsTorqueHardLimitation::Response &res)
{
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsTorqueHardLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request  &req, kortex_driver::GetTwistHardLimitation::Response &res)
{
	Kinova::Api::Base::TwistLimitation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetTwistHardLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request  &req, kortex_driver::GetWrenchHardLimitation::Response &res)
{
	Kinova::Api::Base::WrenchLimitation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetWrenchHardLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::SendJointSpeedsJoystickCommand::Response &res)
{
	Kinova::Api::Base::JointSpeeds input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendJointSpeedsJoystickCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::SendSelectedJointSpeedJoystickCommand::Response &res)
{
	Kinova::Api::Base::JointSpeed input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SendSelectedJointSpeedJoystickCommand(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::EnableBridge(kortex_driver::EnableBridge::Request  &req, kortex_driver::EnableBridge::Response &res)
{
	Kinova::Api::Base::BridgeConfig input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::BridgeResult output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->EnableBridge(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::DisableBridge(kortex_driver::DisableBridge::Request  &req, kortex_driver::DisableBridge::Response &res)
{
	Kinova::Api::Base::BridgeIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::BridgeResult output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->DisableBridge(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetBridgeList(kortex_driver::GetBridgeList::Request  &req, kortex_driver::GetBridgeList::Response &res)
{
	Kinova::Api::Base::BridgeList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetBridgeList(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetBridgeConfig(kortex_driver::GetBridgeConfig::Request  &req, kortex_driver::GetBridgeConfig::Response &res)
{
	Kinova::Api::Base::BridgeIdentifier input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::BridgeConfig output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetBridgeConfig(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::PlayPreComputedJointTrajectory::Response &res)
{
	Kinova::Api::Base::PreComputedJointTrajectory input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->PlayPreComputedJointTrajectory(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetProductConfiguration(kortex_driver::GetProductConfiguration::Request  &req, kortex_driver::GetProductConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::CompleteProductConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetProductConfiguration(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::UpdateDegreeOfFreedomConfiguration(kortex_driver::UpdateDegreeOfFreedomConfiguration::Request  &req, kortex_driver::UpdateDegreeOfFreedomConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationDegreeOfFreedom input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateDegreeOfFreedomConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateBaseTypeConfiguration(kortex_driver::UpdateBaseTypeConfiguration::Request  &req, kortex_driver::UpdateBaseTypeConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationBaseType input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateBaseTypeConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::UpdateEndEffectorTypeConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateEndEffectorTypeConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateVisionModuleTypeConfiguration(kortex_driver::UpdateVisionModuleTypeConfiguration::Request  &req, kortex_driver::UpdateVisionModuleTypeConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationVisionModuleType input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateVisionModuleTypeConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateInterfaceModuleTypeConfiguration(kortex_driver::UpdateInterfaceModuleTypeConfiguration::Request  &req, kortex_driver::UpdateInterfaceModuleTypeConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationInterfaceModuleType input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateInterfaceModuleTypeConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateArmLateralityConfiguration(kortex_driver::UpdateArmLateralityConfiguration::Request  &req, kortex_driver::UpdateArmLateralityConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationLaterality input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateArmLateralityConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::UpdateWristTypeConfiguration(kortex_driver::UpdateWristTypeConfiguration::Request  &req, kortex_driver::UpdateWristTypeConfiguration::Response &res)
{
	Kinova::Api::ProductConfiguration::ProductConfigurationWristType input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateWristTypeConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::RestoreFactoryProductConfiguration::Response &res)
{
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->RestoreFactoryProductConfiguration(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	return true;
}

bool BaseServices::GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request  &req, kortex_driver::GetTrajectoryErrorReport::Response &res)
{
	Kinova::Api::Base::TrajectoryErrorReport output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetTrajectoryErrorReport(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::GetAllJointsSpeedSoftLimitation::Response &res)
{
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsSpeedSoftLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::GetAllJointsTorqueSoftLimitation::Response &res)
{
	Kinova::Api::Base::JointsLimitationsList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllJointsTorqueSoftLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request  &req, kortex_driver::GetTwistSoftLimitation::Response &res)
{
	Kinova::Api::Base::TwistLimitation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetTwistSoftLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}

bool BaseServices::GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request  &req, kortex_driver::GetWrenchSoftLimitation::Response &res)
{
	Kinova::Api::Base::WrenchLimitation output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetWrenchSoftLimitation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.subCode = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error.publish(result_error);
		ROS_INFO("Kortex exception");
		ROS_INFO("KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		ROS_INFO("KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		ROS_INFO("KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		ROS_INFO("%s", ex2.what());
		return false;
	}
	ToRosData(output, res.output);
	return true;
}
