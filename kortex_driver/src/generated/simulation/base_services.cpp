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
 
#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"
#include "kortex_driver/generated/simulation/base_services.h"

BaseSimulationServices::BaseSimulationServices(ros::NodeHandle& node_handle): 
	IBaseServices(node_handle)
{
	m_pub_Error = m_node_handle.advertise<kortex_driver::KortexError>("kortex_error", 1000);
	m_pub_ConfigurationChangeTopic = m_node_handle.advertise<kortex_driver::ConfigurationChangeNotification>("configuration_change_topic", 1000);
	m_is_activated_ConfigurationChangeTopic = false;
	m_pub_MappingInfoTopic = m_node_handle.advertise<kortex_driver::MappingInfoNotification>("mapping_info_topic", 1000);
	m_is_activated_MappingInfoTopic = false;
	m_pub_ControlModeTopic = m_node_handle.advertise<kortex_driver::Base_ControlModeNotification>("control_mode_topic", 1000);
	m_is_activated_ControlModeTopic = false;
	m_pub_OperatingModeTopic = m_node_handle.advertise<kortex_driver::OperatingModeNotification>("operating_mode_topic", 1000);
	m_is_activated_OperatingModeTopic = false;
	m_pub_SequenceInfoTopic = m_node_handle.advertise<kortex_driver::SequenceInfoNotification>("sequence_info_topic", 1000);
	m_is_activated_SequenceInfoTopic = false;
	m_pub_ProtectionZoneTopic = m_node_handle.advertise<kortex_driver::ProtectionZoneNotification>("protection_zone_topic", 1000);
	m_is_activated_ProtectionZoneTopic = false;
	m_pub_UserTopic = m_node_handle.advertise<kortex_driver::UserNotification>("user_topic", 1000);
	m_is_activated_UserTopic = false;
	m_pub_ControllerTopic = m_node_handle.advertise<kortex_driver::ControllerNotification>("controller_topic", 1000);
	m_is_activated_ControllerTopic = false;
	m_pub_ActionTopic = m_node_handle.advertise<kortex_driver::ActionNotification>("action_topic", 1000);
	m_is_activated_ActionTopic = false;
	m_pub_RobotEventTopic = m_node_handle.advertise<kortex_driver::RobotEventNotification>("robot_event_topic", 1000);
	m_is_activated_RobotEventTopic = false;
	m_pub_ServoingModeTopic = m_node_handle.advertise<kortex_driver::ServoingModeNotification>("servoing_mode_topic", 1000);
	m_is_activated_ServoingModeTopic = false;
	m_pub_FactoryTopic = m_node_handle.advertise<kortex_driver::FactoryNotification>("factory_topic", 1000);
	m_is_activated_FactoryTopic = false;
	m_pub_NetworkTopic = m_node_handle.advertise<kortex_driver::NetworkNotification>("network_topic", 1000);
	m_is_activated_NetworkTopic = false;
	m_pub_ArmStateTopic = m_node_handle.advertise<kortex_driver::ArmStateNotification>("arm_state_topic", 1000);
	m_is_activated_ArmStateTopic = false;

	m_serviceSetDeviceID = m_node_handle.advertiseService("base/set_device_id", &BaseSimulationServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("base/set_api_options", &BaseSimulationServices::SetApiOptions, this);

	m_serviceCreateUserProfile = m_node_handle.advertiseService("base/create_user_profile", &BaseSimulationServices::CreateUserProfile, this);
	m_serviceUpdateUserProfile = m_node_handle.advertiseService("base/update_user_profile", &BaseSimulationServices::UpdateUserProfile, this);
	m_serviceReadUserProfile = m_node_handle.advertiseService("base/read_user_profile", &BaseSimulationServices::ReadUserProfile, this);
	m_serviceDeleteUserProfile = m_node_handle.advertiseService("base/delete_user_profile", &BaseSimulationServices::DeleteUserProfile, this);
	m_serviceReadAllUserProfiles = m_node_handle.advertiseService("base/read_all_user_profiles", &BaseSimulationServices::ReadAllUserProfiles, this);
	m_serviceReadAllUsers = m_node_handle.advertiseService("base/read_all_users", &BaseSimulationServices::ReadAllUsers, this);
	m_serviceChangePassword = m_node_handle.advertiseService("base/change_password", &BaseSimulationServices::ChangePassword, this);
	m_serviceCreateSequence = m_node_handle.advertiseService("base/create_sequence", &BaseSimulationServices::CreateSequence, this);
	m_serviceUpdateSequence = m_node_handle.advertiseService("base/update_sequence", &BaseSimulationServices::UpdateSequence, this);
	m_serviceReadSequence = m_node_handle.advertiseService("base/read_sequence", &BaseSimulationServices::ReadSequence, this);
	m_serviceDeleteSequence = m_node_handle.advertiseService("base/delete_sequence", &BaseSimulationServices::DeleteSequence, this);
	m_serviceReadAllSequences = m_node_handle.advertiseService("base/read_all_sequences", &BaseSimulationServices::ReadAllSequences, this);
	m_servicePlaySequence = m_node_handle.advertiseService("base/play_sequence", &BaseSimulationServices::PlaySequence, this);
	m_servicePlayAdvancedSequence = m_node_handle.advertiseService("base/play_advanced_sequence", &BaseSimulationServices::PlayAdvancedSequence, this);
	m_serviceStopSequence = m_node_handle.advertiseService("base/stop_sequence", &BaseSimulationServices::StopSequence, this);
	m_servicePauseSequence = m_node_handle.advertiseService("base/pause_sequence", &BaseSimulationServices::PauseSequence, this);
	m_serviceResumeSequence = m_node_handle.advertiseService("base/resume_sequence", &BaseSimulationServices::ResumeSequence, this);
	m_serviceCreateProtectionZone = m_node_handle.advertiseService("base/create_protection_zone", &BaseSimulationServices::CreateProtectionZone, this);
	m_serviceUpdateProtectionZone = m_node_handle.advertiseService("base/update_protection_zone", &BaseSimulationServices::UpdateProtectionZone, this);
	m_serviceReadProtectionZone = m_node_handle.advertiseService("base/read_protection_zone", &BaseSimulationServices::ReadProtectionZone, this);
	m_serviceDeleteProtectionZone = m_node_handle.advertiseService("base/delete_protection_zone", &BaseSimulationServices::DeleteProtectionZone, this);
	m_serviceReadAllProtectionZones = m_node_handle.advertiseService("base/read_all_protection_zones", &BaseSimulationServices::ReadAllProtectionZones, this);
	m_serviceCreateMapping = m_node_handle.advertiseService("base/create_mapping", &BaseSimulationServices::CreateMapping, this);
	m_serviceReadMapping = m_node_handle.advertiseService("base/read_mapping", &BaseSimulationServices::ReadMapping, this);
	m_serviceUpdateMapping = m_node_handle.advertiseService("base/update_mapping", &BaseSimulationServices::UpdateMapping, this);
	m_serviceDeleteMapping = m_node_handle.advertiseService("base/delete_mapping", &BaseSimulationServices::DeleteMapping, this);
	m_serviceReadAllMappings = m_node_handle.advertiseService("base/read_all_mappings", &BaseSimulationServices::ReadAllMappings, this);
	m_serviceCreateMap = m_node_handle.advertiseService("base/create_map", &BaseSimulationServices::CreateMap, this);
	m_serviceReadMap = m_node_handle.advertiseService("base/read_map", &BaseSimulationServices::ReadMap, this);
	m_serviceUpdateMap = m_node_handle.advertiseService("base/update_map", &BaseSimulationServices::UpdateMap, this);
	m_serviceDeleteMap = m_node_handle.advertiseService("base/delete_map", &BaseSimulationServices::DeleteMap, this);
	m_serviceReadAllMaps = m_node_handle.advertiseService("base/read_all_maps", &BaseSimulationServices::ReadAllMaps, this);
	m_serviceActivateMap = m_node_handle.advertiseService("base/activate_map", &BaseSimulationServices::ActivateMap, this);
	m_serviceCreateAction = m_node_handle.advertiseService("base/create_action", &BaseSimulationServices::CreateAction, this);
	m_serviceReadAction = m_node_handle.advertiseService("base/read_action", &BaseSimulationServices::ReadAction, this);
	m_serviceReadAllActions = m_node_handle.advertiseService("base/read_all_actions", &BaseSimulationServices::ReadAllActions, this);
	m_serviceDeleteAction = m_node_handle.advertiseService("base/delete_action", &BaseSimulationServices::DeleteAction, this);
	m_serviceUpdateAction = m_node_handle.advertiseService("base/update_action", &BaseSimulationServices::UpdateAction, this);
	m_serviceExecuteActionFromReference = m_node_handle.advertiseService("base/execute_action_from_reference", &BaseSimulationServices::ExecuteActionFromReference, this);
	m_serviceExecuteAction = m_node_handle.advertiseService("base/execute_action", &BaseSimulationServices::ExecuteAction, this);
	m_servicePauseAction = m_node_handle.advertiseService("base/pause_action", &BaseSimulationServices::PauseAction, this);
	m_serviceStopAction = m_node_handle.advertiseService("base/stop_action", &BaseSimulationServices::StopAction, this);
	m_serviceResumeAction = m_node_handle.advertiseService("base/resume_action", &BaseSimulationServices::ResumeAction, this);
	m_serviceGetIPv4Configuration = m_node_handle.advertiseService("base/get_i_pv4_configuration", &BaseSimulationServices::GetIPv4Configuration, this);
	m_serviceSetIPv4Configuration = m_node_handle.advertiseService("base/set_i_pv4_configuration", &BaseSimulationServices::SetIPv4Configuration, this);
	m_serviceSetCommunicationInterfaceEnable = m_node_handle.advertiseService("base/set_communication_interface_enable", &BaseSimulationServices::SetCommunicationInterfaceEnable, this);
	m_serviceIsCommunicationInterfaceEnable = m_node_handle.advertiseService("base/is_communication_interface_enable", &BaseSimulationServices::IsCommunicationInterfaceEnable, this);
	m_serviceGetAvailableWifi = m_node_handle.advertiseService("base/get_available_wifi", &BaseSimulationServices::GetAvailableWifi, this);
	m_serviceGetWifiInformation = m_node_handle.advertiseService("base/get_wifi_information", &BaseSimulationServices::GetWifiInformation, this);
	m_serviceAddWifiConfiguration = m_node_handle.advertiseService("base/add_wifi_configuration", &BaseSimulationServices::AddWifiConfiguration, this);
	m_serviceDeleteWifiConfiguration = m_node_handle.advertiseService("base/delete_wifi_configuration", &BaseSimulationServices::DeleteWifiConfiguration, this);
	m_serviceGetAllConfiguredWifis = m_node_handle.advertiseService("base/get_all_configured_wifis", &BaseSimulationServices::GetAllConfiguredWifis, this);
	m_serviceConnectWifi = m_node_handle.advertiseService("base/connect_wifi", &BaseSimulationServices::ConnectWifi, this);
	m_serviceDisconnectWifi = m_node_handle.advertiseService("base/disconnect_wifi", &BaseSimulationServices::DisconnectWifi, this);
	m_serviceGetConnectedWifiInformation = m_node_handle.advertiseService("base/get_connected_wifi_information", &BaseSimulationServices::GetConnectedWifiInformation, this);
	m_serviceBase_Unsubscribe = m_node_handle.advertiseService("base/unsubscribe", &BaseSimulationServices::Base_Unsubscribe, this);
	m_serviceOnNotificationConfigurationChangeTopic = m_node_handle.advertiseService("base/activate_publishing_of_configuration_change_topic", &BaseSimulationServices::OnNotificationConfigurationChangeTopic, this);
	m_serviceOnNotificationMappingInfoTopic = m_node_handle.advertiseService("base/activate_publishing_of_mapping_info_topic", &BaseSimulationServices::OnNotificationMappingInfoTopic, this);
	m_serviceBase_OnNotificationControlModeTopic = m_node_handle.advertiseService("base/activate_publishing_of_control_mode_topic", &BaseSimulationServices::Base_OnNotificationControlModeTopic, this);
	m_serviceOnNotificationOperatingModeTopic = m_node_handle.advertiseService("base/activate_publishing_of_operating_mode_topic", &BaseSimulationServices::OnNotificationOperatingModeTopic, this);
	m_serviceOnNotificationSequenceInfoTopic = m_node_handle.advertiseService("base/activate_publishing_of_sequence_info_topic", &BaseSimulationServices::OnNotificationSequenceInfoTopic, this);
	m_serviceOnNotificationProtectionZoneTopic = m_node_handle.advertiseService("base/activate_publishing_of_protection_zone_topic", &BaseSimulationServices::OnNotificationProtectionZoneTopic, this);
	m_serviceOnNotificationUserTopic = m_node_handle.advertiseService("base/activate_publishing_of_user_topic", &BaseSimulationServices::OnNotificationUserTopic, this);
	m_serviceOnNotificationControllerTopic = m_node_handle.advertiseService("base/activate_publishing_of_controller_topic", &BaseSimulationServices::OnNotificationControllerTopic, this);
	m_serviceOnNotificationActionTopic = m_node_handle.advertiseService("base/activate_publishing_of_action_topic", &BaseSimulationServices::OnNotificationActionTopic, this);
	m_serviceOnNotificationRobotEventTopic = m_node_handle.advertiseService("base/activate_publishing_of_robot_event_topic", &BaseSimulationServices::OnNotificationRobotEventTopic, this);
	m_servicePlayCartesianTrajectory = m_node_handle.advertiseService("base/play_cartesian_trajectory", &BaseSimulationServices::PlayCartesianTrajectory, this);
	m_servicePlayCartesianTrajectoryPosition = m_node_handle.advertiseService("base/play_cartesian_trajectory_position", &BaseSimulationServices::PlayCartesianTrajectoryPosition, this);
	m_servicePlayCartesianTrajectoryOrientation = m_node_handle.advertiseService("base/play_cartesian_trajectory_orientation", &BaseSimulationServices::PlayCartesianTrajectoryOrientation, this);
	m_serviceStop = m_node_handle.advertiseService("base/stop", &BaseSimulationServices::Stop, this);
	m_serviceGetMeasuredCartesianPose = m_node_handle.advertiseService("base/get_measured_cartesian_pose", &BaseSimulationServices::GetMeasuredCartesianPose, this);
	m_serviceSendWrenchCommand = m_node_handle.advertiseService("base/send_wrench_command", &BaseSimulationServices::SendWrenchCommand, this);
	m_serviceSendWrenchJoystickCommand = m_node_handle.advertiseService("base/send_wrench_joystick_command", &BaseSimulationServices::SendWrenchJoystickCommand, this);
	m_serviceSendTwistJoystickCommand = m_node_handle.advertiseService("base/send_twist_joystick_command", &BaseSimulationServices::SendTwistJoystickCommand, this);
	m_serviceSendTwistCommand = m_node_handle.advertiseService("base/send_twist_command", &BaseSimulationServices::SendTwistCommand, this);
	m_servicePlayJointTrajectory = m_node_handle.advertiseService("base/play_joint_trajectory", &BaseSimulationServices::PlayJointTrajectory, this);
	m_servicePlaySelectedJointTrajectory = m_node_handle.advertiseService("base/play_selected_joint_trajectory", &BaseSimulationServices::PlaySelectedJointTrajectory, this);
	m_serviceGetMeasuredJointAngles = m_node_handle.advertiseService("base/get_measured_joint_angles", &BaseSimulationServices::GetMeasuredJointAngles, this);
	m_serviceSendJointSpeedsCommand = m_node_handle.advertiseService("base/send_joint_speeds_command", &BaseSimulationServices::SendJointSpeedsCommand, this);
	m_serviceSendSelectedJointSpeedCommand = m_node_handle.advertiseService("base/send_selected_joint_speed_command", &BaseSimulationServices::SendSelectedJointSpeedCommand, this);
	m_serviceSendGripperCommand = m_node_handle.advertiseService("base/send_gripper_command", &BaseSimulationServices::SendGripperCommand, this);
	m_serviceGetMeasuredGripperMovement = m_node_handle.advertiseService("base/get_measured_gripper_movement", &BaseSimulationServices::GetMeasuredGripperMovement, this);
	m_serviceSetAdmittance = m_node_handle.advertiseService("base/set_admittance", &BaseSimulationServices::SetAdmittance, this);
	m_serviceSetOperatingMode = m_node_handle.advertiseService("base/set_operating_mode", &BaseSimulationServices::SetOperatingMode, this);
	m_serviceApplyEmergencyStop = m_node_handle.advertiseService("base/apply_emergency_stop", &BaseSimulationServices::ApplyEmergencyStop, this);
	m_serviceBase_ClearFaults = m_node_handle.advertiseService("base/clear_faults", &BaseSimulationServices::Base_ClearFaults, this);
	m_serviceBase_GetControlMode = m_node_handle.advertiseService("base/get_control_mode", &BaseSimulationServices::Base_GetControlMode, this);
	m_serviceGetOperatingMode = m_node_handle.advertiseService("base/get_operating_mode", &BaseSimulationServices::GetOperatingMode, this);
	m_serviceSetServoingMode = m_node_handle.advertiseService("base/set_servoing_mode", &BaseSimulationServices::SetServoingMode, this);
	m_serviceGetServoingMode = m_node_handle.advertiseService("base/get_servoing_mode", &BaseSimulationServices::GetServoingMode, this);
	m_serviceOnNotificationServoingModeTopic = m_node_handle.advertiseService("base/activate_publishing_of_servoing_mode_topic", &BaseSimulationServices::OnNotificationServoingModeTopic, this);
	m_serviceRestoreFactorySettings = m_node_handle.advertiseService("base/restore_factory_settings", &BaseSimulationServices::RestoreFactorySettings, this);
	m_serviceOnNotificationFactoryTopic = m_node_handle.advertiseService("base/activate_publishing_of_factory_topic", &BaseSimulationServices::OnNotificationFactoryTopic, this);
	m_serviceGetAllConnectedControllers = m_node_handle.advertiseService("base/get_all_connected_controllers", &BaseSimulationServices::GetAllConnectedControllers, this);
	m_serviceGetControllerState = m_node_handle.advertiseService("base/get_controller_state", &BaseSimulationServices::GetControllerState, this);
	m_serviceGetActuatorCount = m_node_handle.advertiseService("base/get_actuator_count", &BaseSimulationServices::GetActuatorCount, this);
	m_serviceStartWifiScan = m_node_handle.advertiseService("base/start_wifi_scan", &BaseSimulationServices::StartWifiScan, this);
	m_serviceGetConfiguredWifi = m_node_handle.advertiseService("base/get_configured_wifi", &BaseSimulationServices::GetConfiguredWifi, this);
	m_serviceOnNotificationNetworkTopic = m_node_handle.advertiseService("base/activate_publishing_of_network_topic", &BaseSimulationServices::OnNotificationNetworkTopic, this);
	m_serviceGetArmState = m_node_handle.advertiseService("base/get_arm_state", &BaseSimulationServices::GetArmState, this);
	m_serviceOnNotificationArmStateTopic = m_node_handle.advertiseService("base/activate_publishing_of_arm_state_topic", &BaseSimulationServices::OnNotificationArmStateTopic, this);
	m_serviceGetIPv4Information = m_node_handle.advertiseService("base/get_i_pv4_information", &BaseSimulationServices::GetIPv4Information, this);
	m_serviceSetWifiCountryCode = m_node_handle.advertiseService("base/set_wifi_country_code", &BaseSimulationServices::SetWifiCountryCode, this);
	m_serviceGetWifiCountryCode = m_node_handle.advertiseService("base/get_wifi_country_code", &BaseSimulationServices::GetWifiCountryCode, this);
	m_serviceBase_SetCapSenseConfig = m_node_handle.advertiseService("base/set_cap_sense_config", &BaseSimulationServices::Base_SetCapSenseConfig, this);
	m_serviceBase_GetCapSenseConfig = m_node_handle.advertiseService("base/get_cap_sense_config", &BaseSimulationServices::Base_GetCapSenseConfig, this);
	m_serviceGetAllJointsSpeedHardLimitation = m_node_handle.advertiseService("base/get_all_joints_speed_hard_limitation", &BaseSimulationServices::GetAllJointsSpeedHardLimitation, this);
	m_serviceGetAllJointsTorqueHardLimitation = m_node_handle.advertiseService("base/get_all_joints_torque_hard_limitation", &BaseSimulationServices::GetAllJointsTorqueHardLimitation, this);
	m_serviceGetTwistHardLimitation = m_node_handle.advertiseService("base/get_twist_hard_limitation", &BaseSimulationServices::GetTwistHardLimitation, this);
	m_serviceGetWrenchHardLimitation = m_node_handle.advertiseService("base/get_wrench_hard_limitation", &BaseSimulationServices::GetWrenchHardLimitation, this);
	m_serviceSendJointSpeedsJoystickCommand = m_node_handle.advertiseService("base/send_joint_speeds_joystick_command", &BaseSimulationServices::SendJointSpeedsJoystickCommand, this);
	m_serviceSendSelectedJointSpeedJoystickCommand = m_node_handle.advertiseService("base/send_selected_joint_speed_joystick_command", &BaseSimulationServices::SendSelectedJointSpeedJoystickCommand, this);
	m_serviceEnableBridge = m_node_handle.advertiseService("base/enable_bridge", &BaseSimulationServices::EnableBridge, this);
	m_serviceDisableBridge = m_node_handle.advertiseService("base/disable_bridge", &BaseSimulationServices::DisableBridge, this);
	m_serviceGetBridgeList = m_node_handle.advertiseService("base/get_bridge_list", &BaseSimulationServices::GetBridgeList, this);
	m_serviceGetBridgeConfig = m_node_handle.advertiseService("base/get_bridge_config", &BaseSimulationServices::GetBridgeConfig, this);
	m_servicePlayPreComputedJointTrajectory = m_node_handle.advertiseService("base/play_pre_computed_joint_trajectory", &BaseSimulationServices::PlayPreComputedJointTrajectory, this);
	m_serviceGetProductConfiguration = m_node_handle.advertiseService("base/get_product_configuration", &BaseSimulationServices::GetProductConfiguration, this);
	m_serviceUpdateEndEffectorTypeConfiguration = m_node_handle.advertiseService("base/update_end_effector_type_configuration", &BaseSimulationServices::UpdateEndEffectorTypeConfiguration, this);
	m_serviceRestoreFactoryProductConfiguration = m_node_handle.advertiseService("base/restore_factory_product_configuration", &BaseSimulationServices::RestoreFactoryProductConfiguration, this);
	m_serviceGetTrajectoryErrorReport = m_node_handle.advertiseService("base/get_trajectory_error_report", &BaseSimulationServices::GetTrajectoryErrorReport, this);
	m_serviceGetAllJointsSpeedSoftLimitation = m_node_handle.advertiseService("base/get_all_joints_speed_soft_limitation", &BaseSimulationServices::GetAllJointsSpeedSoftLimitation, this);
	m_serviceGetAllJointsTorqueSoftLimitation = m_node_handle.advertiseService("base/get_all_joints_torque_soft_limitation", &BaseSimulationServices::GetAllJointsTorqueSoftLimitation, this);
	m_serviceGetTwistSoftLimitation = m_node_handle.advertiseService("base/get_twist_soft_limitation", &BaseSimulationServices::GetTwistSoftLimitation, this);
	m_serviceGetWrenchSoftLimitation = m_node_handle.advertiseService("base/get_wrench_soft_limitation", &BaseSimulationServices::GetWrenchSoftLimitation, this);
	m_serviceSetControllerConfigurationMode = m_node_handle.advertiseService("base/set_controller_configuration_mode", &BaseSimulationServices::SetControllerConfigurationMode, this);
	m_serviceGetControllerConfigurationMode = m_node_handle.advertiseService("base/get_controller_configuration_mode", &BaseSimulationServices::GetControllerConfigurationMode, this);
	m_serviceStartTeaching = m_node_handle.advertiseService("base/start_teaching", &BaseSimulationServices::StartTeaching, this);
	m_serviceStopTeaching = m_node_handle.advertiseService("base/stop_teaching", &BaseSimulationServices::StopTeaching, this);
	m_serviceAddSequenceTasks = m_node_handle.advertiseService("base/add_sequence_tasks", &BaseSimulationServices::AddSequenceTasks, this);
	m_serviceUpdateSequenceTask = m_node_handle.advertiseService("base/update_sequence_task", &BaseSimulationServices::UpdateSequenceTask, this);
	m_serviceSwapSequenceTasks = m_node_handle.advertiseService("base/swap_sequence_tasks", &BaseSimulationServices::SwapSequenceTasks, this);
	m_serviceReadSequenceTask = m_node_handle.advertiseService("base/read_sequence_task", &BaseSimulationServices::ReadSequenceTask, this);
	m_serviceReadAllSequenceTasks = m_node_handle.advertiseService("base/read_all_sequence_tasks", &BaseSimulationServices::ReadAllSequenceTasks, this);
	m_serviceDeleteSequenceTask = m_node_handle.advertiseService("base/delete_sequence_task", &BaseSimulationServices::DeleteSequenceTask, this);
	m_serviceDeleteAllSequenceTasks = m_node_handle.advertiseService("base/delete_all_sequence_tasks", &BaseSimulationServices::DeleteAllSequenceTasks, this);
	m_serviceTakeSnapshot = m_node_handle.advertiseService("base/take_snapshot", &BaseSimulationServices::TakeSnapshot, this);
	m_serviceGetFirmwareBundleVersions = m_node_handle.advertiseService("base/get_firmware_bundle_versions", &BaseSimulationServices::GetFirmwareBundleVersions, this);
	m_serviceExecuteWaypointTrajectory = m_node_handle.advertiseService("base/execute_waypoint_trajectory", &BaseSimulationServices::ExecuteWaypointTrajectory, this);
	m_serviceMoveSequenceTask = m_node_handle.advertiseService("base/move_sequence_task", &BaseSimulationServices::MoveSequenceTask, this);
	m_serviceDuplicateMapping = m_node_handle.advertiseService("base/duplicate_mapping", &BaseSimulationServices::DuplicateMapping, this);
	m_serviceDuplicateMap = m_node_handle.advertiseService("base/duplicate_map", &BaseSimulationServices::DuplicateMap, this);
	m_serviceSetControllerConfiguration = m_node_handle.advertiseService("base/set_controller_configuration", &BaseSimulationServices::SetControllerConfiguration, this);
	m_serviceGetControllerConfiguration = m_node_handle.advertiseService("base/get_controller_configuration", &BaseSimulationServices::GetControllerConfiguration, this);
	m_serviceGetAllControllerConfigurations = m_node_handle.advertiseService("base/get_all_controller_configurations", &BaseSimulationServices::GetAllControllerConfigurations, this);
	m_serviceComputeForwardKinematics = m_node_handle.advertiseService("base/compute_forward_kinematics", &BaseSimulationServices::ComputeForwardKinematics, this);
	m_serviceComputeInverseKinematics = m_node_handle.advertiseService("base/compute_inverse_kinematics", &BaseSimulationServices::ComputeInverseKinematics, this);
	m_serviceValidateWaypointList = m_node_handle.advertiseService("base/validate_waypoint_list", &BaseSimulationServices::ValidateWaypointList, this);
}

bool BaseSimulationServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool BaseSimulationServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool BaseSimulationServices::CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res)
{
	
	
	if (CreateUserProfileHandler)
	{
		res = CreateUserProfileHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/create_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res)
{
	
	
	if (UpdateUserProfileHandler)
	{
		res = UpdateUserProfileHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res)
{
	
	
	if (ReadUserProfileHandler)
	{
		res = ReadUserProfileHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res)
{
	
	
	if (DeleteUserProfileHandler)
	{
		res = DeleteUserProfileHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_user_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res)
{
	
	
	if (ReadAllUserProfilesHandler)
	{
		res = ReadAllUserProfilesHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_user_profiles is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res)
{
	
	
	if (ReadAllUsersHandler)
	{
		res = ReadAllUsersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_users is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res)
{
	
	
	if (ChangePasswordHandler)
	{
		res = ChangePasswordHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/change_password is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res)
{
	
	
	if (CreateSequenceHandler)
	{
		res = CreateSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/create_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res)
{
	
	
	if (UpdateSequenceHandler)
	{
		res = UpdateSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res)
{
	
	
	if (ReadSequenceHandler)
	{
		res = ReadSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res)
{
	
	
	if (DeleteSequenceHandler)
	{
		res = DeleteSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res)
{
	
	
	if (ReadAllSequencesHandler)
	{
		res = ReadAllSequencesHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_sequences is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res)
{
	
	
	if (PlaySequenceHandler)
	{
		res = PlaySequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res)
{
	
	
	if (PlayAdvancedSequenceHandler)
	{
		res = PlayAdvancedSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_advanced_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res)
{
	
	
	if (StopSequenceHandler)
	{
		res = StopSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/stop_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res)
{
	
	
	if (PauseSequenceHandler)
	{
		res = PauseSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/pause_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res)
{
	
	
	if (ResumeSequenceHandler)
	{
		res = ResumeSequenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/resume_sequence is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res)
{
	
	
	if (CreateProtectionZoneHandler)
	{
		res = CreateProtectionZoneHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/create_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res)
{
	
	
	if (UpdateProtectionZoneHandler)
	{
		res = UpdateProtectionZoneHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res)
{
	
	
	if (ReadProtectionZoneHandler)
	{
		res = ReadProtectionZoneHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res)
{
	
	
	if (DeleteProtectionZoneHandler)
	{
		res = DeleteProtectionZoneHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_protection_zone is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res)
{
	
	
	if (ReadAllProtectionZonesHandler)
	{
		res = ReadAllProtectionZonesHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_protection_zones is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res)
{
	
	
	if (CreateMappingHandler)
	{
		res = CreateMappingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/create_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res)
{
	
	
	if (ReadMappingHandler)
	{
		res = ReadMappingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateMapping(kortex_driver::UpdateMapping::Request  &req, kortex_driver::UpdateMapping::Response &res)
{
	
	
	if (UpdateMappingHandler)
	{
		res = UpdateMappingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteMapping(kortex_driver::DeleteMapping::Request  &req, kortex_driver::DeleteMapping::Response &res)
{
	
	
	if (DeleteMappingHandler)
	{
		res = DeleteMappingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res)
{
	
	
	if (ReadAllMappingsHandler)
	{
		res = ReadAllMappingsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_mappings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res)
{
	
	
	if (CreateMapHandler)
	{
		res = CreateMapHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/create_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadMap(kortex_driver::ReadMap::Request  &req, kortex_driver::ReadMap::Response &res)
{
	
	
	if (ReadMapHandler)
	{
		res = ReadMapHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateMap(kortex_driver::UpdateMap::Request  &req, kortex_driver::UpdateMap::Response &res)
{
	
	
	if (UpdateMapHandler)
	{
		res = UpdateMapHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteMap(kortex_driver::DeleteMap::Request  &req, kortex_driver::DeleteMap::Response &res)
{
	
	
	if (DeleteMapHandler)
	{
		res = DeleteMapHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res)
{
	
	
	if (ReadAllMapsHandler)
	{
		res = ReadAllMapsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_maps is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res)
{
	
	
	if (ActivateMapHandler)
	{
		res = ActivateMapHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res)
{
	
	
	if (CreateActionHandler)
	{
		res = CreateActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/create_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res)
{
	
	
	if (ReadActionHandler)
	{
		res = ReadActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res)
{
	
	
	if (ReadAllActionsHandler)
	{
		res = ReadAllActionsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_actions is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res)
{
	
	
	if (DeleteActionHandler)
	{
		res = DeleteActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res)
{
	
	
	if (UpdateActionHandler)
	{
		res = UpdateActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res)
{
	
	
	if (ExecuteActionFromReferenceHandler)
	{
		res = ExecuteActionFromReferenceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/execute_action_from_reference is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res)
{
	
	
	if (ExecuteActionHandler)
	{
		res = ExecuteActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/execute_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res)
{
	
	
	if (PauseActionHandler)
	{
		res = PauseActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/pause_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res)
{
	
	
	if (StopActionHandler)
	{
		res = StopActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/stop_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res)
{
	
	
	if (ResumeActionHandler)
	{
		res = ResumeActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/resume_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res)
{
	
	
	if (GetIPv4ConfigurationHandler)
	{
		res = GetIPv4ConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_i_pv4_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res)
{
	
	
	if (SetIPv4ConfigurationHandler)
	{
		res = SetIPv4ConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_i_pv4_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res)
{
	
	
	if (SetCommunicationInterfaceEnableHandler)
	{
		res = SetCommunicationInterfaceEnableHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_communication_interface_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res)
{
	
	
	if (IsCommunicationInterfaceEnableHandler)
	{
		res = IsCommunicationInterfaceEnableHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/is_communication_interface_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res)
{
	
	
	if (GetAvailableWifiHandler)
	{
		res = GetAvailableWifiHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_available_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res)
{
	
	
	if (GetWifiInformationHandler)
	{
		res = GetWifiInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_wifi_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res)
{
	
	
	if (AddWifiConfigurationHandler)
	{
		res = AddWifiConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/add_wifi_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res)
{
	
	
	if (DeleteWifiConfigurationHandler)
	{
		res = DeleteWifiConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_wifi_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res)
{
	
	
	if (GetAllConfiguredWifisHandler)
	{
		res = GetAllConfiguredWifisHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_configured_wifis is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res)
{
	
	
	if (ConnectWifiHandler)
	{
		res = ConnectWifiHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/connect_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res)
{
	
	
	if (DisconnectWifiHandler)
	{
		res = DisconnectWifiHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/disconnect_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res)
{
	
	
	if (GetConnectedWifiInformationHandler)
	{
		res = GetConnectedWifiInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_connected_wifi_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request  &req, kortex_driver::Base_Unsubscribe::Response &res)
{
	
	
	if (Base_UnsubscribeHandler)
	{
		res = Base_UnsubscribeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/unsubscribe is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res)
{
	
	m_is_activated_ConfigurationChangeTopic = true;
	
	if (OnNotificationConfigurationChangeTopicHandler)
	{
		res = OnNotificationConfigurationChangeTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_configuration_change_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif)
{
	kortex_driver::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res)
{
	
	m_is_activated_MappingInfoTopic = true;
	
	if (OnNotificationMappingInfoTopicHandler)
	{
		res = OnNotificationMappingInfoTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_mapping_info_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif)
{
	kortex_driver::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic.publish(ros_msg);
}

bool BaseSimulationServices::Base_OnNotificationControlModeTopic(kortex_driver::Base_OnNotificationControlModeTopic::Request  &req, kortex_driver::Base_OnNotificationControlModeTopic::Response &res)
{
	ROS_WARN("The base/activate_publishing_of_control_mode_topic service is now deprecated and will be removed in a future release.");
	
	m_is_activated_ControlModeTopic = true;
	
	if (Base_OnNotificationControlModeTopicHandler)
	{
		res = Base_OnNotificationControlModeTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_control_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif)
{
	kortex_driver::Base_ControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res)
{
	
	m_is_activated_OperatingModeTopic = true;
	
	if (OnNotificationOperatingModeTopicHandler)
	{
		res = OnNotificationOperatingModeTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_operating_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif)
{
	kortex_driver::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res)
{
	
	m_is_activated_SequenceInfoTopic = true;
	
	if (OnNotificationSequenceInfoTopicHandler)
	{
		res = OnNotificationSequenceInfoTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_sequence_info_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif)
{
	kortex_driver::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res)
{
	
	m_is_activated_ProtectionZoneTopic = true;
	
	if (OnNotificationProtectionZoneTopicHandler)
	{
		res = OnNotificationProtectionZoneTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_protection_zone_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif)
{
	kortex_driver::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res)
{
	
	m_is_activated_UserTopic = true;
	
	if (OnNotificationUserTopicHandler)
	{
		res = OnNotificationUserTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_user_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_UserTopic(Kinova::Api::Base::UserNotification notif)
{
	kortex_driver::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res)
{
	
	m_is_activated_ControllerTopic = true;
	
	if (OnNotificationControllerTopicHandler)
	{
		res = OnNotificationControllerTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_controller_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif)
{
	kortex_driver::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res)
{
	
	m_is_activated_ActionTopic = true;
	
	if (OnNotificationActionTopicHandler)
	{
		res = OnNotificationActionTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_action_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ActionTopic(Kinova::Api::Base::ActionNotification notif)
{
	kortex_driver::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic.publish(ros_msg);
}

bool BaseSimulationServices::OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res)
{
	
	m_is_activated_RobotEventTopic = true;
	
	if (OnNotificationRobotEventTopicHandler)
	{
		res = OnNotificationRobotEventTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_robot_event_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif)
{
	kortex_driver::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic.publish(ros_msg);
}

bool BaseSimulationServices::PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory service is now deprecated and will be removed in a future release.");
	
	
	if (PlayCartesianTrajectoryHandler)
	{
		res = PlayCartesianTrajectoryHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_cartesian_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory_position service is now deprecated and will be removed in a future release.");
	
	
	if (PlayCartesianTrajectoryPositionHandler)
	{
		res = PlayCartesianTrajectoryPositionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_cartesian_trajectory_position is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory_orientation service is now deprecated and will be removed in a future release.");
	
	
	if (PlayCartesianTrajectoryOrientationHandler)
	{
		res = PlayCartesianTrajectoryOrientationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_cartesian_trajectory_orientation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Stop(kortex_driver::Stop::Request  &req, kortex_driver::Stop::Response &res)
{
	
	
	if (StopHandler)
	{
		res = StopHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/stop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res)
{
	
	
	if (GetMeasuredCartesianPoseHandler)
	{
		res = GetMeasuredCartesianPoseHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_measured_cartesian_pose is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendWrenchCommand(kortex_driver::SendWrenchCommand::Request  &req, kortex_driver::SendWrenchCommand::Response &res)
{
	
	
	if (SendWrenchCommandHandler)
	{
		res = SendWrenchCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_wrench_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request  &req, kortex_driver::SendWrenchJoystickCommand::Response &res)
{
	
	
	if (SendWrenchJoystickCommandHandler)
	{
		res = SendWrenchJoystickCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_wrench_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request  &req, kortex_driver::SendTwistJoystickCommand::Response &res)
{
	
	
	if (SendTwistJoystickCommandHandler)
	{
		res = SendTwistJoystickCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_twist_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res)
{
	
	
	if (SendTwistCommandHandler)
	{
		res = SendTwistCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_twist_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res)
{
	ROS_WARN("The base/play_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	
	if (PlayJointTrajectoryHandler)
	{
		res = PlayJointTrajectoryHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_joint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res)
{
	ROS_WARN("The base/play_selected_joint_trajectory service is now deprecated and will be removed in a future release.");
	
	
	if (PlaySelectedJointTrajectoryHandler)
	{
		res = PlaySelectedJointTrajectoryHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_selected_joint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res)
{
	
	
	if (GetMeasuredJointAnglesHandler)
	{
		res = GetMeasuredJointAnglesHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_measured_joint_angles is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request  &req, kortex_driver::SendJointSpeedsCommand::Response &res)
{
	
	
	if (SendJointSpeedsCommandHandler)
	{
		res = SendJointSpeedsCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_joint_speeds_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res)
{
	
	
	if (SendSelectedJointSpeedCommandHandler)
	{
		res = SendSelectedJointSpeedCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_selected_joint_speed_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res)
{
	
	
	if (SendGripperCommandHandler)
	{
		res = SendGripperCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_gripper_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res)
{
	
	
	if (GetMeasuredGripperMovementHandler)
	{
		res = GetMeasuredGripperMovementHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_measured_gripper_movement is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res)
{
	
	
	if (SetAdmittanceHandler)
	{
		res = SetAdmittanceHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_admittance is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res)
{
	
	
	if (SetOperatingModeHandler)
	{
		res = SetOperatingModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_operating_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res)
{
	
	
	if (ApplyEmergencyStopHandler)
	{
		res = ApplyEmergencyStopHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/apply_emergency_stop is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_ClearFaults(kortex_driver::Base_ClearFaults::Request  &req, kortex_driver::Base_ClearFaults::Response &res)
{
	
	
	if (Base_ClearFaultsHandler)
	{
		res = Base_ClearFaultsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/clear_faults is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_GetControlMode(kortex_driver::Base_GetControlMode::Request  &req, kortex_driver::Base_GetControlMode::Response &res)
{
	ROS_WARN("The base/get_control_mode service is now deprecated and will be removed in a future release.");
	
	
	if (Base_GetControlModeHandler)
	{
		res = Base_GetControlModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_control_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res)
{
	
	
	if (GetOperatingModeHandler)
	{
		res = GetOperatingModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_operating_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res)
{
	
	
	if (SetServoingModeHandler)
	{
		res = SetServoingModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_servoing_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res)
{
	
	
	if (GetServoingModeHandler)
	{
		res = GetServoingModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_servoing_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res)
{
	
	m_is_activated_ServoingModeTopic = true;
	
	if (OnNotificationServoingModeTopicHandler)
	{
		res = OnNotificationServoingModeTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_servoing_mode_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif)
{
	kortex_driver::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic.publish(ros_msg);
}

bool BaseSimulationServices::RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res)
{
	
	
	if (RestoreFactorySettingsHandler)
	{
		res = RestoreFactorySettingsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/restore_factory_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res)
{
	
	m_is_activated_FactoryTopic = true;
	
	if (OnNotificationFactoryTopicHandler)
	{
		res = OnNotificationFactoryTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_factory_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif)
{
	kortex_driver::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic.publish(ros_msg);
}

bool BaseSimulationServices::GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res)
{
	
	
	if (GetAllConnectedControllersHandler)
	{
		res = GetAllConnectedControllersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_connected_controllers is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res)
{
	
	
	if (GetControllerStateHandler)
	{
		res = GetControllerStateHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_controller_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res)
{
	
	
	if (GetActuatorCountHandler)
	{
		res = GetActuatorCountHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_actuator_count is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res)
{
	
	
	if (StartWifiScanHandler)
	{
		res = StartWifiScanHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/start_wifi_scan is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res)
{
	
	
	if (GetConfiguredWifiHandler)
	{
		res = GetConfiguredWifiHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_configured_wifi is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res)
{
	
	m_is_activated_NetworkTopic = true;
	
	if (OnNotificationNetworkTopicHandler)
	{
		res = OnNotificationNetworkTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_network_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif)
{
	kortex_driver::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic.publish(ros_msg);
}

bool BaseSimulationServices::GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res)
{
	
	
	if (GetArmStateHandler)
	{
		res = GetArmStateHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_arm_state is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res)
{
	
	m_is_activated_ArmStateTopic = true;
	
	if (OnNotificationArmStateTopicHandler)
	{
		res = OnNotificationArmStateTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/activate_publishing_of_arm_state_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void BaseSimulationServices::cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif)
{
	kortex_driver::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic.publish(ros_msg);
}

bool BaseSimulationServices::GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res)
{
	
	
	if (GetIPv4InformationHandler)
	{
		res = GetIPv4InformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_i_pv4_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request  &req, kortex_driver::SetWifiCountryCode::Response &res)
{
	
	
	if (SetWifiCountryCodeHandler)
	{
		res = SetWifiCountryCodeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_wifi_country_code is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request  &req, kortex_driver::GetWifiCountryCode::Response &res)
{
	
	
	if (GetWifiCountryCodeHandler)
	{
		res = GetWifiCountryCodeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_wifi_country_code is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request  &req, kortex_driver::Base_SetCapSenseConfig::Response &res)
{
	
	
	if (Base_SetCapSenseConfigHandler)
	{
		res = Base_SetCapSenseConfigHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request  &req, kortex_driver::Base_GetCapSenseConfig::Response &res)
{
	
	
	if (Base_GetCapSenseConfigHandler)
	{
		res = Base_GetCapSenseConfigHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::GetAllJointsSpeedHardLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_speed_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsSpeedHardLimitationHandler)
	{
		res = GetAllJointsSpeedHardLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_joints_speed_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::GetAllJointsTorqueHardLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_torque_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsTorqueHardLimitationHandler)
	{
		res = GetAllJointsTorqueHardLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_joints_torque_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request  &req, kortex_driver::GetTwistHardLimitation::Response &res)
{
	ROS_WARN("The base/get_twist_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetTwistHardLimitationHandler)
	{
		res = GetTwistHardLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_twist_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request  &req, kortex_driver::GetWrenchHardLimitation::Response &res)
{
	ROS_WARN("The base/get_wrench_hard_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetWrenchHardLimitationHandler)
	{
		res = GetWrenchHardLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_wrench_hard_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::SendJointSpeedsJoystickCommand::Response &res)
{
	
	
	if (SendJointSpeedsJoystickCommandHandler)
	{
		res = SendJointSpeedsJoystickCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_joint_speeds_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::SendSelectedJointSpeedJoystickCommand::Response &res)
{
	
	
	if (SendSelectedJointSpeedJoystickCommandHandler)
	{
		res = SendSelectedJointSpeedJoystickCommandHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/send_selected_joint_speed_joystick_command is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::EnableBridge(kortex_driver::EnableBridge::Request  &req, kortex_driver::EnableBridge::Response &res)
{
	
	
	if (EnableBridgeHandler)
	{
		res = EnableBridgeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/enable_bridge is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DisableBridge(kortex_driver::DisableBridge::Request  &req, kortex_driver::DisableBridge::Response &res)
{
	
	
	if (DisableBridgeHandler)
	{
		res = DisableBridgeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/disable_bridge is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetBridgeList(kortex_driver::GetBridgeList::Request  &req, kortex_driver::GetBridgeList::Response &res)
{
	
	
	if (GetBridgeListHandler)
	{
		res = GetBridgeListHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_bridge_list is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetBridgeConfig(kortex_driver::GetBridgeConfig::Request  &req, kortex_driver::GetBridgeConfig::Response &res)
{
	
	
	if (GetBridgeConfigHandler)
	{
		res = GetBridgeConfigHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_bridge_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::PlayPreComputedJointTrajectory::Response &res)
{
	
	
	if (PlayPreComputedJointTrajectoryHandler)
	{
		res = PlayPreComputedJointTrajectoryHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/play_pre_computed_joint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetProductConfiguration(kortex_driver::GetProductConfiguration::Request  &req, kortex_driver::GetProductConfiguration::Response &res)
{
	
	
	if (GetProductConfigurationHandler)
	{
		res = GetProductConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_product_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::UpdateEndEffectorTypeConfiguration::Response &res)
{
	
	
	if (UpdateEndEffectorTypeConfigurationHandler)
	{
		res = UpdateEndEffectorTypeConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_end_effector_type_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::RestoreFactoryProductConfiguration::Response &res)
{
	
	
	if (RestoreFactoryProductConfigurationHandler)
	{
		res = RestoreFactoryProductConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/restore_factory_product_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request  &req, kortex_driver::GetTrajectoryErrorReport::Response &res)
{
	
	
	if (GetTrajectoryErrorReportHandler)
	{
		res = GetTrajectoryErrorReportHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_trajectory_error_report is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::GetAllJointsSpeedSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_speed_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsSpeedSoftLimitationHandler)
	{
		res = GetAllJointsSpeedSoftLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_joints_speed_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::GetAllJointsTorqueSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_torque_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetAllJointsTorqueSoftLimitationHandler)
	{
		res = GetAllJointsTorqueSoftLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_joints_torque_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request  &req, kortex_driver::GetTwistSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_twist_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetTwistSoftLimitationHandler)
	{
		res = GetTwistSoftLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_twist_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request  &req, kortex_driver::GetWrenchSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_wrench_soft_limitation service is now deprecated and will be removed in a future release.");
	
	
	if (GetWrenchSoftLimitationHandler)
	{
		res = GetWrenchSoftLimitationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_wrench_soft_limitation is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetControllerConfigurationMode(kortex_driver::SetControllerConfigurationMode::Request  &req, kortex_driver::SetControllerConfigurationMode::Response &res)
{
	
	
	if (SetControllerConfigurationModeHandler)
	{
		res = SetControllerConfigurationModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_controller_configuration_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetControllerConfigurationMode(kortex_driver::GetControllerConfigurationMode::Request  &req, kortex_driver::GetControllerConfigurationMode::Response &res)
{
	
	
	if (GetControllerConfigurationModeHandler)
	{
		res = GetControllerConfigurationModeHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_controller_configuration_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StartTeaching(kortex_driver::StartTeaching::Request  &req, kortex_driver::StartTeaching::Response &res)
{
	
	
	if (StartTeachingHandler)
	{
		res = StartTeachingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/start_teaching is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::StopTeaching(kortex_driver::StopTeaching::Request  &req, kortex_driver::StopTeaching::Response &res)
{
	
	
	if (StopTeachingHandler)
	{
		res = StopTeachingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/stop_teaching is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::AddSequenceTasks(kortex_driver::AddSequenceTasks::Request  &req, kortex_driver::AddSequenceTasks::Response &res)
{
	
	
	if (AddSequenceTasksHandler)
	{
		res = AddSequenceTasksHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/add_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::UpdateSequenceTask(kortex_driver::UpdateSequenceTask::Request  &req, kortex_driver::UpdateSequenceTask::Response &res)
{
	
	
	if (UpdateSequenceTaskHandler)
	{
		res = UpdateSequenceTaskHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/update_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SwapSequenceTasks(kortex_driver::SwapSequenceTasks::Request  &req, kortex_driver::SwapSequenceTasks::Response &res)
{
	
	
	if (SwapSequenceTasksHandler)
	{
		res = SwapSequenceTasksHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/swap_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadSequenceTask(kortex_driver::ReadSequenceTask::Request  &req, kortex_driver::ReadSequenceTask::Response &res)
{
	
	
	if (ReadSequenceTaskHandler)
	{
		res = ReadSequenceTaskHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ReadAllSequenceTasks(kortex_driver::ReadAllSequenceTasks::Request  &req, kortex_driver::ReadAllSequenceTasks::Response &res)
{
	
	
	if (ReadAllSequenceTasksHandler)
	{
		res = ReadAllSequenceTasksHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/read_all_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res)
{
	
	
	if (DeleteSequenceTaskHandler)
	{
		res = DeleteSequenceTaskHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res)
{
	
	
	if (DeleteAllSequenceTasksHandler)
	{
		res = DeleteAllSequenceTasksHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/delete_all_sequence_tasks is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::TakeSnapshot(kortex_driver::TakeSnapshot::Request  &req, kortex_driver::TakeSnapshot::Response &res)
{
	
	
	if (TakeSnapshotHandler)
	{
		res = TakeSnapshotHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/take_snapshot is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetFirmwareBundleVersions(kortex_driver::GetFirmwareBundleVersions::Request  &req, kortex_driver::GetFirmwareBundleVersions::Response &res)
{
	
	
	if (GetFirmwareBundleVersionsHandler)
	{
		res = GetFirmwareBundleVersionsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_firmware_bundle_versions is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ExecuteWaypointTrajectory(kortex_driver::ExecuteWaypointTrajectory::Request  &req, kortex_driver::ExecuteWaypointTrajectory::Response &res)
{
	
	
	if (ExecuteWaypointTrajectoryHandler)
	{
		res = ExecuteWaypointTrajectoryHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/execute_waypoint_trajectory is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::MoveSequenceTask(kortex_driver::MoveSequenceTask::Request  &req, kortex_driver::MoveSequenceTask::Response &res)
{
	
	
	if (MoveSequenceTaskHandler)
	{
		res = MoveSequenceTaskHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/move_sequence_task is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DuplicateMapping(kortex_driver::DuplicateMapping::Request  &req, kortex_driver::DuplicateMapping::Response &res)
{
	
	
	if (DuplicateMappingHandler)
	{
		res = DuplicateMappingHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/duplicate_mapping is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::DuplicateMap(kortex_driver::DuplicateMap::Request  &req, kortex_driver::DuplicateMap::Response &res)
{
	
	
	if (DuplicateMapHandler)
	{
		res = DuplicateMapHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/duplicate_map is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::SetControllerConfiguration(kortex_driver::SetControllerConfiguration::Request  &req, kortex_driver::SetControllerConfiguration::Response &res)
{
	
	
	if (SetControllerConfigurationHandler)
	{
		res = SetControllerConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/set_controller_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetControllerConfiguration(kortex_driver::GetControllerConfiguration::Request  &req, kortex_driver::GetControllerConfiguration::Response &res)
{
	
	
	if (GetControllerConfigurationHandler)
	{
		res = GetControllerConfigurationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_controller_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::GetAllControllerConfigurations(kortex_driver::GetAllControllerConfigurations::Request  &req, kortex_driver::GetAllControllerConfigurations::Response &res)
{
	
	
	if (GetAllControllerConfigurationsHandler)
	{
		res = GetAllControllerConfigurationsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/get_all_controller_configurations is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ComputeForwardKinematics(kortex_driver::ComputeForwardKinematics::Request  &req, kortex_driver::ComputeForwardKinematics::Response &res)
{
	
	
	if (ComputeForwardKinematicsHandler)
	{
		res = ComputeForwardKinematicsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/compute_forward_kinematics is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ComputeInverseKinematics(kortex_driver::ComputeInverseKinematics::Request  &req, kortex_driver::ComputeInverseKinematics::Response &res)
{
	
	
	if (ComputeInverseKinematicsHandler)
	{
		res = ComputeInverseKinematicsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/compute_inverse_kinematics is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool BaseSimulationServices::ValidateWaypointList(kortex_driver::ValidateWaypointList::Request  &req, kortex_driver::ValidateWaypointList::Response &res)
{
	
	
	if (ValidateWaypointListHandler)
	{
		res = ValidateWaypointListHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for base/validate_waypoint_list is not implemented, so the service calls will return the default response.");
	}
	return true;
}
