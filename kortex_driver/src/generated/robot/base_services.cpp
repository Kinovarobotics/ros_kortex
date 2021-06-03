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
#include "kortex_driver/generated/robot/base_services.h"

BaseRobotServices::BaseRobotServices(ros::NodeHandle& node_handle, Kinova::Api::Base::BaseClient* base, uint32_t device_id, uint32_t timeout_ms): 
	IBaseServices(node_handle),
	m_base(base),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

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

	m_serviceSetDeviceID = m_node_handle.advertiseService("base/set_device_id", &BaseRobotServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle.advertiseService("base/set_api_options", &BaseRobotServices::SetApiOptions, this);

	m_serviceCreateUserProfile = m_node_handle.advertiseService("base/create_user_profile", &BaseRobotServices::CreateUserProfile, this);
	m_serviceUpdateUserProfile = m_node_handle.advertiseService("base/update_user_profile", &BaseRobotServices::UpdateUserProfile, this);
	m_serviceReadUserProfile = m_node_handle.advertiseService("base/read_user_profile", &BaseRobotServices::ReadUserProfile, this);
	m_serviceDeleteUserProfile = m_node_handle.advertiseService("base/delete_user_profile", &BaseRobotServices::DeleteUserProfile, this);
	m_serviceReadAllUserProfiles = m_node_handle.advertiseService("base/read_all_user_profiles", &BaseRobotServices::ReadAllUserProfiles, this);
	m_serviceReadAllUsers = m_node_handle.advertiseService("base/read_all_users", &BaseRobotServices::ReadAllUsers, this);
	m_serviceChangePassword = m_node_handle.advertiseService("base/change_password", &BaseRobotServices::ChangePassword, this);
	m_serviceCreateSequence = m_node_handle.advertiseService("base/create_sequence", &BaseRobotServices::CreateSequence, this);
	m_serviceUpdateSequence = m_node_handle.advertiseService("base/update_sequence", &BaseRobotServices::UpdateSequence, this);
	m_serviceReadSequence = m_node_handle.advertiseService("base/read_sequence", &BaseRobotServices::ReadSequence, this);
	m_serviceDeleteSequence = m_node_handle.advertiseService("base/delete_sequence", &BaseRobotServices::DeleteSequence, this);
	m_serviceReadAllSequences = m_node_handle.advertiseService("base/read_all_sequences", &BaseRobotServices::ReadAllSequences, this);
	m_servicePlaySequence = m_node_handle.advertiseService("base/play_sequence", &BaseRobotServices::PlaySequence, this);
	m_servicePlayAdvancedSequence = m_node_handle.advertiseService("base/play_advanced_sequence", &BaseRobotServices::PlayAdvancedSequence, this);
	m_serviceStopSequence = m_node_handle.advertiseService("base/stop_sequence", &BaseRobotServices::StopSequence, this);
	m_servicePauseSequence = m_node_handle.advertiseService("base/pause_sequence", &BaseRobotServices::PauseSequence, this);
	m_serviceResumeSequence = m_node_handle.advertiseService("base/resume_sequence", &BaseRobotServices::ResumeSequence, this);
	m_serviceCreateProtectionZone = m_node_handle.advertiseService("base/create_protection_zone", &BaseRobotServices::CreateProtectionZone, this);
	m_serviceUpdateProtectionZone = m_node_handle.advertiseService("base/update_protection_zone", &BaseRobotServices::UpdateProtectionZone, this);
	m_serviceReadProtectionZone = m_node_handle.advertiseService("base/read_protection_zone", &BaseRobotServices::ReadProtectionZone, this);
	m_serviceDeleteProtectionZone = m_node_handle.advertiseService("base/delete_protection_zone", &BaseRobotServices::DeleteProtectionZone, this);
	m_serviceReadAllProtectionZones = m_node_handle.advertiseService("base/read_all_protection_zones", &BaseRobotServices::ReadAllProtectionZones, this);
	m_serviceCreateMapping = m_node_handle.advertiseService("base/create_mapping", &BaseRobotServices::CreateMapping, this);
	m_serviceReadMapping = m_node_handle.advertiseService("base/read_mapping", &BaseRobotServices::ReadMapping, this);
	m_serviceUpdateMapping = m_node_handle.advertiseService("base/update_mapping", &BaseRobotServices::UpdateMapping, this);
	m_serviceDeleteMapping = m_node_handle.advertiseService("base/delete_mapping", &BaseRobotServices::DeleteMapping, this);
	m_serviceReadAllMappings = m_node_handle.advertiseService("base/read_all_mappings", &BaseRobotServices::ReadAllMappings, this);
	m_serviceCreateMap = m_node_handle.advertiseService("base/create_map", &BaseRobotServices::CreateMap, this);
	m_serviceReadMap = m_node_handle.advertiseService("base/read_map", &BaseRobotServices::ReadMap, this);
	m_serviceUpdateMap = m_node_handle.advertiseService("base/update_map", &BaseRobotServices::UpdateMap, this);
	m_serviceDeleteMap = m_node_handle.advertiseService("base/delete_map", &BaseRobotServices::DeleteMap, this);
	m_serviceReadAllMaps = m_node_handle.advertiseService("base/read_all_maps", &BaseRobotServices::ReadAllMaps, this);
	m_serviceActivateMap = m_node_handle.advertiseService("base/activate_map", &BaseRobotServices::ActivateMap, this);
	m_serviceCreateAction = m_node_handle.advertiseService("base/create_action", &BaseRobotServices::CreateAction, this);
	m_serviceReadAction = m_node_handle.advertiseService("base/read_action", &BaseRobotServices::ReadAction, this);
	m_serviceReadAllActions = m_node_handle.advertiseService("base/read_all_actions", &BaseRobotServices::ReadAllActions, this);
	m_serviceDeleteAction = m_node_handle.advertiseService("base/delete_action", &BaseRobotServices::DeleteAction, this);
	m_serviceUpdateAction = m_node_handle.advertiseService("base/update_action", &BaseRobotServices::UpdateAction, this);
	m_serviceExecuteActionFromReference = m_node_handle.advertiseService("base/execute_action_from_reference", &BaseRobotServices::ExecuteActionFromReference, this);
	m_serviceExecuteAction = m_node_handle.advertiseService("base/execute_action", &BaseRobotServices::ExecuteAction, this);
	m_servicePauseAction = m_node_handle.advertiseService("base/pause_action", &BaseRobotServices::PauseAction, this);
	m_serviceStopAction = m_node_handle.advertiseService("base/stop_action", &BaseRobotServices::StopAction, this);
	m_serviceResumeAction = m_node_handle.advertiseService("base/resume_action", &BaseRobotServices::ResumeAction, this);
	m_serviceGetIPv4Configuration = m_node_handle.advertiseService("base/get_i_pv4_configuration", &BaseRobotServices::GetIPv4Configuration, this);
	m_serviceSetIPv4Configuration = m_node_handle.advertiseService("base/set_i_pv4_configuration", &BaseRobotServices::SetIPv4Configuration, this);
	m_serviceSetCommunicationInterfaceEnable = m_node_handle.advertiseService("base/set_communication_interface_enable", &BaseRobotServices::SetCommunicationInterfaceEnable, this);
	m_serviceIsCommunicationInterfaceEnable = m_node_handle.advertiseService("base/is_communication_interface_enable", &BaseRobotServices::IsCommunicationInterfaceEnable, this);
	m_serviceGetAvailableWifi = m_node_handle.advertiseService("base/get_available_wifi", &BaseRobotServices::GetAvailableWifi, this);
	m_serviceGetWifiInformation = m_node_handle.advertiseService("base/get_wifi_information", &BaseRobotServices::GetWifiInformation, this);
	m_serviceAddWifiConfiguration = m_node_handle.advertiseService("base/add_wifi_configuration", &BaseRobotServices::AddWifiConfiguration, this);
	m_serviceDeleteWifiConfiguration = m_node_handle.advertiseService("base/delete_wifi_configuration", &BaseRobotServices::DeleteWifiConfiguration, this);
	m_serviceGetAllConfiguredWifis = m_node_handle.advertiseService("base/get_all_configured_wifis", &BaseRobotServices::GetAllConfiguredWifis, this);
	m_serviceConnectWifi = m_node_handle.advertiseService("base/connect_wifi", &BaseRobotServices::ConnectWifi, this);
	m_serviceDisconnectWifi = m_node_handle.advertiseService("base/disconnect_wifi", &BaseRobotServices::DisconnectWifi, this);
	m_serviceGetConnectedWifiInformation = m_node_handle.advertiseService("base/get_connected_wifi_information", &BaseRobotServices::GetConnectedWifiInformation, this);
	m_serviceBase_Unsubscribe = m_node_handle.advertiseService("base/unsubscribe", &BaseRobotServices::Base_Unsubscribe, this);
	m_serviceOnNotificationConfigurationChangeTopic = m_node_handle.advertiseService("base/activate_publishing_of_configuration_change_topic", &BaseRobotServices::OnNotificationConfigurationChangeTopic, this);
	m_serviceOnNotificationMappingInfoTopic = m_node_handle.advertiseService("base/activate_publishing_of_mapping_info_topic", &BaseRobotServices::OnNotificationMappingInfoTopic, this);
	m_serviceBase_OnNotificationControlModeTopic = m_node_handle.advertiseService("base/activate_publishing_of_control_mode_topic", &BaseRobotServices::Base_OnNotificationControlModeTopic, this);
	m_serviceOnNotificationOperatingModeTopic = m_node_handle.advertiseService("base/activate_publishing_of_operating_mode_topic", &BaseRobotServices::OnNotificationOperatingModeTopic, this);
	m_serviceOnNotificationSequenceInfoTopic = m_node_handle.advertiseService("base/activate_publishing_of_sequence_info_topic", &BaseRobotServices::OnNotificationSequenceInfoTopic, this);
	m_serviceOnNotificationProtectionZoneTopic = m_node_handle.advertiseService("base/activate_publishing_of_protection_zone_topic", &BaseRobotServices::OnNotificationProtectionZoneTopic, this);
	m_serviceOnNotificationUserTopic = m_node_handle.advertiseService("base/activate_publishing_of_user_topic", &BaseRobotServices::OnNotificationUserTopic, this);
	m_serviceOnNotificationControllerTopic = m_node_handle.advertiseService("base/activate_publishing_of_controller_topic", &BaseRobotServices::OnNotificationControllerTopic, this);
	m_serviceOnNotificationActionTopic = m_node_handle.advertiseService("base/activate_publishing_of_action_topic", &BaseRobotServices::OnNotificationActionTopic, this);
	m_serviceOnNotificationRobotEventTopic = m_node_handle.advertiseService("base/activate_publishing_of_robot_event_topic", &BaseRobotServices::OnNotificationRobotEventTopic, this);
	m_servicePlayCartesianTrajectory = m_node_handle.advertiseService("base/play_cartesian_trajectory", &BaseRobotServices::PlayCartesianTrajectory, this);
	m_servicePlayCartesianTrajectoryPosition = m_node_handle.advertiseService("base/play_cartesian_trajectory_position", &BaseRobotServices::PlayCartesianTrajectoryPosition, this);
	m_servicePlayCartesianTrajectoryOrientation = m_node_handle.advertiseService("base/play_cartesian_trajectory_orientation", &BaseRobotServices::PlayCartesianTrajectoryOrientation, this);
	m_serviceStop = m_node_handle.advertiseService("base/stop", &BaseRobotServices::Stop, this);
	m_serviceGetMeasuredCartesianPose = m_node_handle.advertiseService("base/get_measured_cartesian_pose", &BaseRobotServices::GetMeasuredCartesianPose, this);
	m_serviceSendWrenchCommand = m_node_handle.advertiseService("base/send_wrench_command", &BaseRobotServices::SendWrenchCommand, this);
	m_serviceSendWrenchJoystickCommand = m_node_handle.advertiseService("base/send_wrench_joystick_command", &BaseRobotServices::SendWrenchJoystickCommand, this);
	m_serviceSendTwistJoystickCommand = m_node_handle.advertiseService("base/send_twist_joystick_command", &BaseRobotServices::SendTwistJoystickCommand, this);
	m_serviceSendTwistCommand = m_node_handle.advertiseService("base/send_twist_command", &BaseRobotServices::SendTwistCommand, this);
	m_servicePlayJointTrajectory = m_node_handle.advertiseService("base/play_joint_trajectory", &BaseRobotServices::PlayJointTrajectory, this);
	m_servicePlaySelectedJointTrajectory = m_node_handle.advertiseService("base/play_selected_joint_trajectory", &BaseRobotServices::PlaySelectedJointTrajectory, this);
	m_serviceGetMeasuredJointAngles = m_node_handle.advertiseService("base/get_measured_joint_angles", &BaseRobotServices::GetMeasuredJointAngles, this);
	m_serviceSendJointSpeedsCommand = m_node_handle.advertiseService("base/send_joint_speeds_command", &BaseRobotServices::SendJointSpeedsCommand, this);
	m_serviceSendSelectedJointSpeedCommand = m_node_handle.advertiseService("base/send_selected_joint_speed_command", &BaseRobotServices::SendSelectedJointSpeedCommand, this);
	m_serviceSendGripperCommand = m_node_handle.advertiseService("base/send_gripper_command", &BaseRobotServices::SendGripperCommand, this);
	m_serviceGetMeasuredGripperMovement = m_node_handle.advertiseService("base/get_measured_gripper_movement", &BaseRobotServices::GetMeasuredGripperMovement, this);
	m_serviceSetAdmittance = m_node_handle.advertiseService("base/set_admittance", &BaseRobotServices::SetAdmittance, this);
	m_serviceSetOperatingMode = m_node_handle.advertiseService("base/set_operating_mode", &BaseRobotServices::SetOperatingMode, this);
	m_serviceApplyEmergencyStop = m_node_handle.advertiseService("base/apply_emergency_stop", &BaseRobotServices::ApplyEmergencyStop, this);
	m_serviceBase_ClearFaults = m_node_handle.advertiseService("base/clear_faults", &BaseRobotServices::Base_ClearFaults, this);
	m_serviceBase_GetControlMode = m_node_handle.advertiseService("base/get_control_mode", &BaseRobotServices::Base_GetControlMode, this);
	m_serviceGetOperatingMode = m_node_handle.advertiseService("base/get_operating_mode", &BaseRobotServices::GetOperatingMode, this);
	m_serviceSetServoingMode = m_node_handle.advertiseService("base/set_servoing_mode", &BaseRobotServices::SetServoingMode, this);
	m_serviceGetServoingMode = m_node_handle.advertiseService("base/get_servoing_mode", &BaseRobotServices::GetServoingMode, this);
	m_serviceOnNotificationServoingModeTopic = m_node_handle.advertiseService("base/activate_publishing_of_servoing_mode_topic", &BaseRobotServices::OnNotificationServoingModeTopic, this);
	m_serviceRestoreFactorySettings = m_node_handle.advertiseService("base/restore_factory_settings", &BaseRobotServices::RestoreFactorySettings, this);
	m_serviceOnNotificationFactoryTopic = m_node_handle.advertiseService("base/activate_publishing_of_factory_topic", &BaseRobotServices::OnNotificationFactoryTopic, this);
	m_serviceGetAllConnectedControllers = m_node_handle.advertiseService("base/get_all_connected_controllers", &BaseRobotServices::GetAllConnectedControllers, this);
	m_serviceGetControllerState = m_node_handle.advertiseService("base/get_controller_state", &BaseRobotServices::GetControllerState, this);
	m_serviceGetActuatorCount = m_node_handle.advertiseService("base/get_actuator_count", &BaseRobotServices::GetActuatorCount, this);
	m_serviceStartWifiScan = m_node_handle.advertiseService("base/start_wifi_scan", &BaseRobotServices::StartWifiScan, this);
	m_serviceGetConfiguredWifi = m_node_handle.advertiseService("base/get_configured_wifi", &BaseRobotServices::GetConfiguredWifi, this);
	m_serviceOnNotificationNetworkTopic = m_node_handle.advertiseService("base/activate_publishing_of_network_topic", &BaseRobotServices::OnNotificationNetworkTopic, this);
	m_serviceGetArmState = m_node_handle.advertiseService("base/get_arm_state", &BaseRobotServices::GetArmState, this);
	m_serviceOnNotificationArmStateTopic = m_node_handle.advertiseService("base/activate_publishing_of_arm_state_topic", &BaseRobotServices::OnNotificationArmStateTopic, this);
	m_serviceGetIPv4Information = m_node_handle.advertiseService("base/get_i_pv4_information", &BaseRobotServices::GetIPv4Information, this);
	m_serviceSetWifiCountryCode = m_node_handle.advertiseService("base/set_wifi_country_code", &BaseRobotServices::SetWifiCountryCode, this);
	m_serviceGetWifiCountryCode = m_node_handle.advertiseService("base/get_wifi_country_code", &BaseRobotServices::GetWifiCountryCode, this);
	m_serviceBase_SetCapSenseConfig = m_node_handle.advertiseService("base/set_cap_sense_config", &BaseRobotServices::Base_SetCapSenseConfig, this);
	m_serviceBase_GetCapSenseConfig = m_node_handle.advertiseService("base/get_cap_sense_config", &BaseRobotServices::Base_GetCapSenseConfig, this);
	m_serviceGetAllJointsSpeedHardLimitation = m_node_handle.advertiseService("base/get_all_joints_speed_hard_limitation", &BaseRobotServices::GetAllJointsSpeedHardLimitation, this);
	m_serviceGetAllJointsTorqueHardLimitation = m_node_handle.advertiseService("base/get_all_joints_torque_hard_limitation", &BaseRobotServices::GetAllJointsTorqueHardLimitation, this);
	m_serviceGetTwistHardLimitation = m_node_handle.advertiseService("base/get_twist_hard_limitation", &BaseRobotServices::GetTwistHardLimitation, this);
	m_serviceGetWrenchHardLimitation = m_node_handle.advertiseService("base/get_wrench_hard_limitation", &BaseRobotServices::GetWrenchHardLimitation, this);
	m_serviceSendJointSpeedsJoystickCommand = m_node_handle.advertiseService("base/send_joint_speeds_joystick_command", &BaseRobotServices::SendJointSpeedsJoystickCommand, this);
	m_serviceSendSelectedJointSpeedJoystickCommand = m_node_handle.advertiseService("base/send_selected_joint_speed_joystick_command", &BaseRobotServices::SendSelectedJointSpeedJoystickCommand, this);
	m_serviceEnableBridge = m_node_handle.advertiseService("base/enable_bridge", &BaseRobotServices::EnableBridge, this);
	m_serviceDisableBridge = m_node_handle.advertiseService("base/disable_bridge", &BaseRobotServices::DisableBridge, this);
	m_serviceGetBridgeList = m_node_handle.advertiseService("base/get_bridge_list", &BaseRobotServices::GetBridgeList, this);
	m_serviceGetBridgeConfig = m_node_handle.advertiseService("base/get_bridge_config", &BaseRobotServices::GetBridgeConfig, this);
	m_servicePlayPreComputedJointTrajectory = m_node_handle.advertiseService("base/play_pre_computed_joint_trajectory", &BaseRobotServices::PlayPreComputedJointTrajectory, this);
	m_serviceGetProductConfiguration = m_node_handle.advertiseService("base/get_product_configuration", &BaseRobotServices::GetProductConfiguration, this);
	m_serviceUpdateEndEffectorTypeConfiguration = m_node_handle.advertiseService("base/update_end_effector_type_configuration", &BaseRobotServices::UpdateEndEffectorTypeConfiguration, this);
	m_serviceRestoreFactoryProductConfiguration = m_node_handle.advertiseService("base/restore_factory_product_configuration", &BaseRobotServices::RestoreFactoryProductConfiguration, this);
	m_serviceGetTrajectoryErrorReport = m_node_handle.advertiseService("base/get_trajectory_error_report", &BaseRobotServices::GetTrajectoryErrorReport, this);
	m_serviceGetAllJointsSpeedSoftLimitation = m_node_handle.advertiseService("base/get_all_joints_speed_soft_limitation", &BaseRobotServices::GetAllJointsSpeedSoftLimitation, this);
	m_serviceGetAllJointsTorqueSoftLimitation = m_node_handle.advertiseService("base/get_all_joints_torque_soft_limitation", &BaseRobotServices::GetAllJointsTorqueSoftLimitation, this);
	m_serviceGetTwistSoftLimitation = m_node_handle.advertiseService("base/get_twist_soft_limitation", &BaseRobotServices::GetTwistSoftLimitation, this);
	m_serviceGetWrenchSoftLimitation = m_node_handle.advertiseService("base/get_wrench_soft_limitation", &BaseRobotServices::GetWrenchSoftLimitation, this);
	m_serviceSetControllerConfigurationMode = m_node_handle.advertiseService("base/set_controller_configuration_mode", &BaseRobotServices::SetControllerConfigurationMode, this);
	m_serviceGetControllerConfigurationMode = m_node_handle.advertiseService("base/get_controller_configuration_mode", &BaseRobotServices::GetControllerConfigurationMode, this);
	m_serviceStartTeaching = m_node_handle.advertiseService("base/start_teaching", &BaseRobotServices::StartTeaching, this);
	m_serviceStopTeaching = m_node_handle.advertiseService("base/stop_teaching", &BaseRobotServices::StopTeaching, this);
	m_serviceAddSequenceTasks = m_node_handle.advertiseService("base/add_sequence_tasks", &BaseRobotServices::AddSequenceTasks, this);
	m_serviceUpdateSequenceTask = m_node_handle.advertiseService("base/update_sequence_task", &BaseRobotServices::UpdateSequenceTask, this);
	m_serviceSwapSequenceTasks = m_node_handle.advertiseService("base/swap_sequence_tasks", &BaseRobotServices::SwapSequenceTasks, this);
	m_serviceReadSequenceTask = m_node_handle.advertiseService("base/read_sequence_task", &BaseRobotServices::ReadSequenceTask, this);
	m_serviceReadAllSequenceTasks = m_node_handle.advertiseService("base/read_all_sequence_tasks", &BaseRobotServices::ReadAllSequenceTasks, this);
	m_serviceDeleteSequenceTask = m_node_handle.advertiseService("base/delete_sequence_task", &BaseRobotServices::DeleteSequenceTask, this);
	m_serviceDeleteAllSequenceTasks = m_node_handle.advertiseService("base/delete_all_sequence_tasks", &BaseRobotServices::DeleteAllSequenceTasks, this);
	m_serviceTakeSnapshot = m_node_handle.advertiseService("base/take_snapshot", &BaseRobotServices::TakeSnapshot, this);
	m_serviceGetFirmwareBundleVersions = m_node_handle.advertiseService("base/get_firmware_bundle_versions", &BaseRobotServices::GetFirmwareBundleVersions, this);
	m_serviceExecuteWaypointTrajectory = m_node_handle.advertiseService("base/execute_waypoint_trajectory", &BaseRobotServices::ExecuteWaypointTrajectory, this);
	m_serviceMoveSequenceTask = m_node_handle.advertiseService("base/move_sequence_task", &BaseRobotServices::MoveSequenceTask, this);
	m_serviceDuplicateMapping = m_node_handle.advertiseService("base/duplicate_mapping", &BaseRobotServices::DuplicateMapping, this);
	m_serviceDuplicateMap = m_node_handle.advertiseService("base/duplicate_map", &BaseRobotServices::DuplicateMap, this);
	m_serviceSetControllerConfiguration = m_node_handle.advertiseService("base/set_controller_configuration", &BaseRobotServices::SetControllerConfiguration, this);
	m_serviceGetControllerConfiguration = m_node_handle.advertiseService("base/get_controller_configuration", &BaseRobotServices::GetControllerConfiguration, this);
	m_serviceGetAllControllerConfigurations = m_node_handle.advertiseService("base/get_all_controller_configurations", &BaseRobotServices::GetAllControllerConfigurations, this);
	m_serviceComputeForwardKinematics = m_node_handle.advertiseService("base/compute_forward_kinematics", &BaseRobotServices::ComputeForwardKinematics, this);
	m_serviceComputeInverseKinematics = m_node_handle.advertiseService("base/compute_inverse_kinematics", &BaseRobotServices::ComputeInverseKinematics, this);
	m_serviceValidateWaypointList = m_node_handle.advertiseService("base/validate_waypoint_list", &BaseRobotServices::ValidateWaypointList, this);
}

bool BaseRobotServices::SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool BaseRobotServices::SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool BaseRobotServices::CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res)
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

bool BaseRobotServices::UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res)
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

bool BaseRobotServices::ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res)
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

bool BaseRobotServices::DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res)
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

bool BaseRobotServices::ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res)
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

bool BaseRobotServices::ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res)
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

bool BaseRobotServices::ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res)
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

bool BaseRobotServices::CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res)
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

bool BaseRobotServices::UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res)
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

bool BaseRobotServices::ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res)
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

bool BaseRobotServices::DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res)
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

bool BaseRobotServices::ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res)
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

bool BaseRobotServices::PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res)
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

bool BaseRobotServices::PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res)
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

bool BaseRobotServices::StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res)
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

bool BaseRobotServices::PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res)
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

bool BaseRobotServices::ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res)
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

bool BaseRobotServices::CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res)
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

bool BaseRobotServices::UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res)
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

bool BaseRobotServices::ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res)
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

bool BaseRobotServices::DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res)
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

bool BaseRobotServices::ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res)
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

bool BaseRobotServices::CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res)
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

bool BaseRobotServices::ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res)
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

bool BaseRobotServices::UpdateMapping(kortex_driver::UpdateMapping::Request  &req, kortex_driver::UpdateMapping::Response &res)
{
	
	Kinova::Api::Base::Mapping input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteMapping(kortex_driver::DeleteMapping::Request  &req, kortex_driver::DeleteMapping::Response &res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res)
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

bool BaseRobotServices::CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res)
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

bool BaseRobotServices::ReadMap(kortex_driver::ReadMap::Request  &req, kortex_driver::ReadMap::Response &res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::Map output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateMap(kortex_driver::UpdateMap::Request  &req, kortex_driver::UpdateMap::Response &res)
{
	
	Kinova::Api::Base::Map input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteMap(kortex_driver::DeleteMap::Request  &req, kortex_driver::DeleteMap::Response &res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->DeleteMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res)
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

bool BaseRobotServices::ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res)
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

bool BaseRobotServices::CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res)
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

bool BaseRobotServices::ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res)
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

bool BaseRobotServices::ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res)
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

bool BaseRobotServices::DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res)
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

bool BaseRobotServices::UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res)
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

bool BaseRobotServices::ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res)
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

bool BaseRobotServices::ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res)
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

bool BaseRobotServices::PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res)
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

bool BaseRobotServices::StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res)
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

bool BaseRobotServices::ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res)
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

bool BaseRobotServices::GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res)
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

bool BaseRobotServices::SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res)
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

bool BaseRobotServices::SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res)
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

bool BaseRobotServices::IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res)
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

bool BaseRobotServices::GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res)
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

bool BaseRobotServices::GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res)
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

bool BaseRobotServices::AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res)
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

bool BaseRobotServices::DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res)
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

bool BaseRobotServices::GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res)
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

bool BaseRobotServices::ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res)
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

bool BaseRobotServices::DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res)
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

bool BaseRobotServices::GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res)
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

bool BaseRobotServices::Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request  &req, kortex_driver::Base_Unsubscribe::Response &res)
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

bool BaseRobotServices::OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ConfigurationChangeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ConfigurationChangeNotification) > callback = std::bind(&BaseRobotServices::cb_ConfigurationChangeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationConfigurationChangeTopic(callback, input, m_current_device_id);
		m_is_activated_ConfigurationChangeTopic = true;
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
void BaseRobotServices::cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif)
{
	kortex_driver::ConfigurationChangeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ConfigurationChangeTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_MappingInfoTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::MappingInfoNotification) > callback = std::bind(&BaseRobotServices::cb_MappingInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationMappingInfoTopic(callback, input, m_current_device_id);
		m_is_activated_MappingInfoTopic = true;
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
void BaseRobotServices::cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif)
{
	kortex_driver::MappingInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_MappingInfoTopic.publish(ros_msg);
}

bool BaseRobotServices::Base_OnNotificationControlModeTopic(kortex_driver::Base_OnNotificationControlModeTopic::Request  &req, kortex_driver::Base_OnNotificationControlModeTopic::Response &res)
{
	ROS_WARN("The base/activate_publishing_of_control_mode_topic service is now deprecated and will be removed in a future release.");
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ControlModeNotification) > callback = std::bind(&BaseRobotServices::cb_ControlModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControlModeTopic(callback, input, m_current_device_id);
		m_is_activated_ControlModeTopic = true;
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
void BaseRobotServices::cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif)
{
	kortex_driver::Base_ControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_OperatingModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::OperatingModeNotification) > callback = std::bind(&BaseRobotServices::cb_OperatingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationOperatingModeTopic(callback, input, m_current_device_id);
		m_is_activated_OperatingModeTopic = true;
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
void BaseRobotServices::cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif)
{
	kortex_driver::OperatingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_OperatingModeTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_SequenceInfoTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::SequenceInfoNotification) > callback = std::bind(&BaseRobotServices::cb_SequenceInfoTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationSequenceInfoTopic(callback, input, m_current_device_id);
		m_is_activated_SequenceInfoTopic = true;
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
void BaseRobotServices::cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif)
{
	kortex_driver::SequenceInfoNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SequenceInfoTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ProtectionZoneTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ProtectionZoneNotification) > callback = std::bind(&BaseRobotServices::cb_ProtectionZoneTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationProtectionZoneTopic(callback, input, m_current_device_id);
		m_is_activated_ProtectionZoneTopic = true;
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
void BaseRobotServices::cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif)
{
	kortex_driver::ProtectionZoneNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ProtectionZoneTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_UserTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::UserNotification) > callback = std::bind(&BaseRobotServices::cb_UserTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationUserTopic(callback, input, m_current_device_id);
		m_is_activated_UserTopic = true;
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
void BaseRobotServices::cb_UserTopic(Kinova::Api::Base::UserNotification notif)
{
	kortex_driver::UserNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_UserTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControllerTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ControllerNotification) > callback = std::bind(&BaseRobotServices::cb_ControllerTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationControllerTopic(callback, input, m_current_device_id);
		m_is_activated_ControllerTopic = true;
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
void BaseRobotServices::cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif)
{
	kortex_driver::ControllerNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControllerTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ActionTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ActionNotification) > callback = std::bind(&BaseRobotServices::cb_ActionTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationActionTopic(callback, input, m_current_device_id);
		m_is_activated_ActionTopic = true;
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
void BaseRobotServices::cb_ActionTopic(Kinova::Api::Base::ActionNotification notif)
{
	kortex_driver::ActionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ActionTopic.publish(ros_msg);
}

bool BaseRobotServices::OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_RobotEventTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::RobotEventNotification) > callback = std::bind(&BaseRobotServices::cb_RobotEventTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationRobotEventTopic(callback, input, m_current_device_id);
		m_is_activated_RobotEventTopic = true;
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
void BaseRobotServices::cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif)
{
	kortex_driver::RobotEventNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_RobotEventTopic.publish(ros_msg);
}

bool BaseRobotServices::PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory_position service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res)
{
	ROS_WARN("The base/play_cartesian_trajectory_orientation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::Stop(kortex_driver::Stop::Request  &req, kortex_driver::Stop::Response &res)
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

bool BaseRobotServices::GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res)
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

bool BaseRobotServices::SendWrenchCommand(kortex_driver::SendWrenchCommand::Request  &req, kortex_driver::SendWrenchCommand::Response &res)
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

bool BaseRobotServices::SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request  &req, kortex_driver::SendWrenchJoystickCommand::Response &res)
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

bool BaseRobotServices::SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request  &req, kortex_driver::SendTwistJoystickCommand::Response &res)
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

bool BaseRobotServices::SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res)
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

bool BaseRobotServices::PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res)
{
	ROS_WARN("The base/play_joint_trajectory service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res)
{
	ROS_WARN("The base/play_selected_joint_trajectory service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res)
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

bool BaseRobotServices::SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request  &req, kortex_driver::SendJointSpeedsCommand::Response &res)
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

bool BaseRobotServices::SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res)
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

bool BaseRobotServices::SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res)
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

bool BaseRobotServices::GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res)
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

bool BaseRobotServices::SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res)
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

bool BaseRobotServices::SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res)
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

bool BaseRobotServices::ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res)
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

bool BaseRobotServices::Base_ClearFaults(kortex_driver::Base_ClearFaults::Request  &req, kortex_driver::Base_ClearFaults::Response &res)
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

bool BaseRobotServices::Base_GetControlMode(kortex_driver::Base_GetControlMode::Request  &req, kortex_driver::Base_GetControlMode::Response &res)
{
	ROS_WARN("The base/get_control_mode service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res)
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

bool BaseRobotServices::SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res)
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

bool BaseRobotServices::GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res)
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

bool BaseRobotServices::OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ServoingModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ServoingModeNotification) > callback = std::bind(&BaseRobotServices::cb_ServoingModeTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationServoingModeTopic(callback, input, m_current_device_id);
		m_is_activated_ServoingModeTopic = true;
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
void BaseRobotServices::cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif)
{
	kortex_driver::ServoingModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ServoingModeTopic.publish(ros_msg);
}

bool BaseRobotServices::RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res)
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

bool BaseRobotServices::OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_FactoryTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::FactoryNotification) > callback = std::bind(&BaseRobotServices::cb_FactoryTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationFactoryTopic(callback, input, m_current_device_id);
		m_is_activated_FactoryTopic = true;
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
void BaseRobotServices::cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif)
{
	kortex_driver::FactoryNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_FactoryTopic.publish(ros_msg);
}

bool BaseRobotServices::GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res)
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

bool BaseRobotServices::GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res)
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

bool BaseRobotServices::GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res)
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

bool BaseRobotServices::StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res)
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

bool BaseRobotServices::GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res)
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

bool BaseRobotServices::OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_NetworkTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::NetworkNotification) > callback = std::bind(&BaseRobotServices::cb_NetworkTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationNetworkTopic(callback, input, m_current_device_id);
		m_is_activated_NetworkTopic = true;
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
void BaseRobotServices::cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif)
{
	kortex_driver::NetworkNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_NetworkTopic.publish(ros_msg);
}

bool BaseRobotServices::GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res)
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

bool BaseRobotServices::OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ArmStateTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Base::ArmStateNotification) > callback = std::bind(&BaseRobotServices::cb_ArmStateTopic, this, std::placeholders::_1);
		output = m_base->OnNotificationArmStateTopic(callback, input, m_current_device_id);
		m_is_activated_ArmStateTopic = true;
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
void BaseRobotServices::cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif)
{
	kortex_driver::ArmStateNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ArmStateTopic.publish(ros_msg);
}

bool BaseRobotServices::GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res)
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

bool BaseRobotServices::SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request  &req, kortex_driver::SetWifiCountryCode::Response &res)
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

bool BaseRobotServices::GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request  &req, kortex_driver::GetWifiCountryCode::Response &res)
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

bool BaseRobotServices::Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request  &req, kortex_driver::Base_SetCapSenseConfig::Response &res)
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

bool BaseRobotServices::Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request  &req, kortex_driver::Base_GetCapSenseConfig::Response &res)
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

bool BaseRobotServices::GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::GetAllJointsSpeedHardLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_speed_hard_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::GetAllJointsTorqueHardLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_torque_hard_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request  &req, kortex_driver::GetTwistHardLimitation::Response &res)
{
	ROS_WARN("The base/get_twist_hard_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request  &req, kortex_driver::GetWrenchHardLimitation::Response &res)
{
	ROS_WARN("The base/get_wrench_hard_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::SendJointSpeedsJoystickCommand::Response &res)
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

bool BaseRobotServices::SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::SendSelectedJointSpeedJoystickCommand::Response &res)
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

bool BaseRobotServices::EnableBridge(kortex_driver::EnableBridge::Request  &req, kortex_driver::EnableBridge::Response &res)
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

bool BaseRobotServices::DisableBridge(kortex_driver::DisableBridge::Request  &req, kortex_driver::DisableBridge::Response &res)
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

bool BaseRobotServices::GetBridgeList(kortex_driver::GetBridgeList::Request  &req, kortex_driver::GetBridgeList::Response &res)
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

bool BaseRobotServices::GetBridgeConfig(kortex_driver::GetBridgeConfig::Request  &req, kortex_driver::GetBridgeConfig::Response &res)
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

bool BaseRobotServices::PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::PlayPreComputedJointTrajectory::Response &res)
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

bool BaseRobotServices::GetProductConfiguration(kortex_driver::GetProductConfiguration::Request  &req, kortex_driver::GetProductConfiguration::Response &res)
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

bool BaseRobotServices::UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::UpdateEndEffectorTypeConfiguration::Response &res)
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

bool BaseRobotServices::RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::RestoreFactoryProductConfiguration::Response &res)
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

bool BaseRobotServices::GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request  &req, kortex_driver::GetTrajectoryErrorReport::Response &res)
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

bool BaseRobotServices::GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::GetAllJointsSpeedSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_speed_soft_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::GetAllJointsTorqueSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_all_joints_torque_soft_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request  &req, kortex_driver::GetTwistSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_twist_soft_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request  &req, kortex_driver::GetWrenchSoftLimitation::Response &res)
{
	ROS_WARN("The base/get_wrench_soft_limitation service is now deprecated and will be removed in a future release.");
	
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

bool BaseRobotServices::SetControllerConfigurationMode(kortex_driver::SetControllerConfigurationMode::Request  &req, kortex_driver::SetControllerConfigurationMode::Response &res)
{
	
	Kinova::Api::Base::ControllerConfigurationMode input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetControllerConfigurationMode(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetControllerConfigurationMode(kortex_driver::GetControllerConfigurationMode::Request  &req, kortex_driver::GetControllerConfigurationMode::Response &res)
{
	
	Kinova::Api::Base::ControllerConfigurationMode output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerConfigurationMode(m_current_device_id, m_api_options);
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

bool BaseRobotServices::StartTeaching(kortex_driver::StartTeaching::Request  &req, kortex_driver::StartTeaching::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StartTeaching(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::StopTeaching(kortex_driver::StopTeaching::Request  &req, kortex_driver::StopTeaching::Response &res)
{
	
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->StopTeaching(m_current_device_id, m_api_options);
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

bool BaseRobotServices::AddSequenceTasks(kortex_driver::AddSequenceTasks::Request  &req, kortex_driver::AddSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceTasksConfiguration input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::SequenceTasksRange output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->AddSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::UpdateSequenceTask(kortex_driver::UpdateSequenceTask::Request  &req, kortex_driver::UpdateSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->UpdateSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SwapSequenceTasks(kortex_driver::SwapSequenceTasks::Request  &req, kortex_driver::SwapSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceTasksPair input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SwapSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadSequenceTask(kortex_driver::ReadSequenceTask::Request  &req, kortex_driver::ReadSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTaskHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::SequenceTask output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ReadAllSequenceTasks(kortex_driver::ReadAllSequenceTasks::Request  &req, kortex_driver::ReadAllSequenceTasks::Response &res)
{
	
	Kinova::Api::Base::SequenceHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::SequenceTasks output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ReadAllSequenceTasks(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res)
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

bool BaseRobotServices::DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res)
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

bool BaseRobotServices::TakeSnapshot(kortex_driver::TakeSnapshot::Request  &req, kortex_driver::TakeSnapshot::Response &res)
{
	
	Kinova::Api::Base::Snapshot input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->TakeSnapshot(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetFirmwareBundleVersions(kortex_driver::GetFirmwareBundleVersions::Request  &req, kortex_driver::GetFirmwareBundleVersions::Response &res)
{
	
	Kinova::Api::Base::FirmwareBundleVersions output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetFirmwareBundleVersions(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ExecuteWaypointTrajectory(kortex_driver::ExecuteWaypointTrajectory::Request  &req, kortex_driver::ExecuteWaypointTrajectory::Response &res)
{
	
	Kinova::Api::Base::WaypointList input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->ExecuteWaypointTrajectory(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::MoveSequenceTask(kortex_driver::MoveSequenceTask::Request  &req, kortex_driver::MoveSequenceTask::Response &res)
{
	
	Kinova::Api::Base::SequenceTasksPair input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->MoveSequenceTask(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DuplicateMapping(kortex_driver::DuplicateMapping::Request  &req, kortex_driver::DuplicateMapping::Response &res)
{
	
	Kinova::Api::Base::MappingHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::MappingHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->DuplicateMapping(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::DuplicateMap(kortex_driver::DuplicateMap::Request  &req, kortex_driver::DuplicateMap::Response &res)
{
	
	Kinova::Api::Base::MapHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::MapHandle output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->DuplicateMap(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::SetControllerConfiguration(kortex_driver::SetControllerConfiguration::Request  &req, kortex_driver::SetControllerConfiguration::Response &res)
{
	
	Kinova::Api::Base::ControllerConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::KortexError result_error;
	
	try
	{
		m_base->SetControllerConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetControllerConfiguration(kortex_driver::GetControllerConfiguration::Request  &req, kortex_driver::GetControllerConfiguration::Response &res)
{
	
	Kinova::Api::Base::ControllerHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::ControllerConfiguration output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetControllerConfiguration(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::GetAllControllerConfigurations(kortex_driver::GetAllControllerConfigurations::Request  &req, kortex_driver::GetAllControllerConfigurations::Response &res)
{
	
	Kinova::Api::Base::ControllerConfigurationList output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->GetAllControllerConfigurations(m_current_device_id, m_api_options);
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

bool BaseRobotServices::ComputeForwardKinematics(kortex_driver::ComputeForwardKinematics::Request  &req, kortex_driver::ComputeForwardKinematics::Response &res)
{
	
	Kinova::Api::Base::JointAngles input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::Pose output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ComputeForwardKinematics(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ComputeInverseKinematics(kortex_driver::ComputeInverseKinematics::Request  &req, kortex_driver::ComputeInverseKinematics::Response &res)
{
	
	Kinova::Api::Base::IKData input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::JointAngles output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ComputeInverseKinematics(input, m_current_device_id, m_api_options);
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

bool BaseRobotServices::ValidateWaypointList(kortex_driver::ValidateWaypointList::Request  &req, kortex_driver::ValidateWaypointList::Response &res)
{
	
	Kinova::Api::Base::WaypointList input;
	ToProtoData(req.input, &input);
	Kinova::Api::Base::WaypointValidationReport output;
	
	kortex_driver::KortexError result_error;
	
	try
	{
		output = m_base->ValidateWaypointList(input, m_current_device_id, m_api_options);
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
