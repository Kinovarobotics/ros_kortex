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
 
#ifndef _KORTEX_BASE_SERVICES_H_
#define _KORTEX_BASE_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Base.pb.h>
#include <BaseClientRpc.h>
#include "kortex_driver/CreateUserProfile.h"
#include "kortex_driver/UpdateUserProfile.h"
#include "kortex_driver/ReadUserProfile.h"
#include "kortex_driver/DeleteUserProfile.h"
#include "kortex_driver/ReadAllUserProfiles.h"
#include "kortex_driver/ReadAllUsers.h"
#include "kortex_driver/ChangePassword.h"
#include "kortex_driver/CreateSequence.h"
#include "kortex_driver/UpdateSequence.h"
#include "kortex_driver/ReadSequence.h"
#include "kortex_driver/DeleteSequence.h"
#include "kortex_driver/ReadAllSequences.h"
#include "kortex_driver/DeleteSequenceTask.h"
#include "kortex_driver/DeleteAllSequenceTasks.h"
#include "kortex_driver/PlaySequence.h"
#include "kortex_driver/PlayAdvancedSequence.h"
#include "kortex_driver/StopSequence.h"
#include "kortex_driver/PauseSequence.h"
#include "kortex_driver/ResumeSequence.h"
#include "kortex_driver/CreateProtectionZone.h"
#include "kortex_driver/UpdateProtectionZone.h"
#include "kortex_driver/ReadProtectionZone.h"
#include "kortex_driver/DeleteProtectionZone.h"
#include "kortex_driver/ReadAllProtectionZones.h"
#include "kortex_driver/CreateMapping.h"
#include "kortex_driver/ReadMapping.h"
#include "kortex_driver/ReadAllMappings.h"
#include "kortex_driver/CreateMap.h"
#include "kortex_driver/ReadAllMaps.h"
#include "kortex_driver/ActivateMap.h"
#include "kortex_driver/CreateAction.h"
#include "kortex_driver/ReadAction.h"
#include "kortex_driver/ReadAllActions.h"
#include "kortex_driver/DeleteAction.h"
#include "kortex_driver/UpdateAction.h"
#include "kortex_driver/ExecuteActionFromReference.h"
#include "kortex_driver/ExecuteAction.h"
#include "kortex_driver/PauseAction.h"
#include "kortex_driver/StopAction.h"
#include "kortex_driver/ResumeAction.h"
#include "kortex_driver/GetIPv4Configuration.h"
#include "kortex_driver/SetIPv4Configuration.h"
#include "kortex_driver/SetCommunicationInterfaceEnable.h"
#include "kortex_driver/IsCommunicationInterfaceEnable.h"
#include "kortex_driver/GetAvailableWifi.h"
#include "kortex_driver/GetWifiInformation.h"
#include "kortex_driver/AddWifiConfiguration.h"
#include "kortex_driver/DeleteWifiConfiguration.h"
#include "kortex_driver/GetAllConfiguredWifis.h"
#include "kortex_driver/ConnectWifi.h"
#include "kortex_driver/DisconnectWifi.h"
#include "kortex_driver/GetConnectedWifiInformation.h"
#include "kortex_driver/Base_Unsubscribe.h"
#include "kortex_driver/OnNotificationConfigurationChangeTopic.h"
#include "kortex_driver/ConfigurationChangeNotification.h"
#include "kortex_driver/OnNotificationMappingInfoTopic.h"
#include "kortex_driver/MappingInfoNotification.h"
#include "kortex_driver/OnNotificationControlModeTopic.h"
#include "kortex_driver/ControlModeNotification.h"
#include "kortex_driver/OnNotificationOperatingModeTopic.h"
#include "kortex_driver/OperatingModeNotification.h"
#include "kortex_driver/OnNotificationSequenceInfoTopic.h"
#include "kortex_driver/SequenceInfoNotification.h"
#include "kortex_driver/OnNotificationProtectionZoneTopic.h"
#include "kortex_driver/ProtectionZoneNotification.h"
#include "kortex_driver/OnNotificationUserTopic.h"
#include "kortex_driver/UserNotification.h"
#include "kortex_driver/OnNotificationControllerTopic.h"
#include "kortex_driver/ControllerNotification.h"
#include "kortex_driver/OnNotificationActionTopic.h"
#include "kortex_driver/ActionNotification.h"
#include "kortex_driver/OnNotificationRobotEventTopic.h"
#include "kortex_driver/RobotEventNotification.h"
#include "kortex_driver/PlayCartesianTrajectory.h"
#include "kortex_driver/PlayCartesianTrajectoryPosition.h"
#include "kortex_driver/PlayCartesianTrajectoryOrientation.h"
#include "kortex_driver/Stop.h"
#include "kortex_driver/GetMeasuredCartesianPose.h"
#include "kortex_driver/SendWrenchCommand.h"
#include "kortex_driver/SendWrenchJoystickCommand.h"
#include "kortex_driver/SendTwistJoystickCommand.h"
#include "kortex_driver/SendTwistCommand.h"
#include "kortex_driver/PlayJointTrajectory.h"
#include "kortex_driver/PlaySelectedJointTrajectory.h"
#include "kortex_driver/GetMeasuredJointAngles.h"
#include "kortex_driver/SendJointSpeedsCommand.h"
#include "kortex_driver/SendSelectedJointSpeedCommand.h"
#include "kortex_driver/SendGripperCommand.h"
#include "kortex_driver/GetMeasuredGripperMovement.h"
#include "kortex_driver/SetAdmittance.h"
#include "kortex_driver/SetOperatingMode.h"
#include "kortex_driver/ApplyEmergencyStop.h"
#include "kortex_driver/Base_ClearFaults.h"
#include "kortex_driver/Base_GetControlMode.h"
#include "kortex_driver/GetOperatingMode.h"
#include "kortex_driver/SetServoingMode.h"
#include "kortex_driver/GetServoingMode.h"
#include "kortex_driver/OnNotificationServoingModeTopic.h"
#include "kortex_driver/ServoingModeNotification.h"
#include "kortex_driver/RestoreFactorySettings.h"
#include "kortex_driver/OnNotificationFactoryTopic.h"
#include "kortex_driver/FactoryNotification.h"
#include "kortex_driver/GetAllConnectedControllers.h"
#include "kortex_driver/GetControllerState.h"
#include "kortex_driver/GetActuatorCount.h"
#include "kortex_driver/StartWifiScan.h"
#include "kortex_driver/GetConfiguredWifi.h"
#include "kortex_driver/OnNotificationNetworkTopic.h"
#include "kortex_driver/NetworkNotification.h"
#include "kortex_driver/GetArmState.h"
#include "kortex_driver/OnNotificationArmStateTopic.h"
#include "kortex_driver/ArmStateNotification.h"
#include "kortex_driver/GetIPv4Information.h"
#include "kortex_driver/SetWifiCountryCode.h"
#include "kortex_driver/GetWifiCountryCode.h"
#include "kortex_driver/Base_SetCapSenseConfig.h"
#include "kortex_driver/Base_GetCapSenseConfig.h"
#include "kortex_driver/GetAllJointsSpeedHardLimitation.h"
#include "kortex_driver/GetAllJointsTorqueHardLimitation.h"
#include "kortex_driver/GetTwistHardLimitation.h"
#include "kortex_driver/GetWrenchHardLimitation.h"
#include "kortex_driver/SendJointSpeedsJoystickCommand.h"
#include "kortex_driver/SendSelectedJointSpeedJoystickCommand.h"
#include "kortex_driver/EnableBridge.h"
#include "kortex_driver/DisableBridge.h"
#include "kortex_driver/GetBridgeList.h"
#include "kortex_driver/GetBridgeConfig.h"
#include "kortex_driver/PlayPreComputedJointTrajectory.h"
#include "kortex_driver/GetProductConfiguration.h"
#include "kortex_driver/UpdateDegreeOfFreedomConfiguration.h"
#include "kortex_driver/UpdateBaseTypeConfiguration.h"
#include "kortex_driver/UpdateEndEffectorTypeConfiguration.h"
#include "kortex_driver/UpdateVisionModuleTypeConfiguration.h"
#include "kortex_driver/UpdateInterfaceModuleTypeConfiguration.h"
#include "kortex_driver/UpdateArmLateralityConfiguration.h"
#include "kortex_driver/UpdateWristTypeConfiguration.h"
#include "kortex_driver/RestoreFactoryProductConfiguration.h"
#include "kortex_driver/GetTrajectoryErrorReport.h"
#include "kortex_driver/GetAllJointsSpeedSoftLimitation.h"
#include "kortex_driver/GetAllJointsTorqueSoftLimitation.h"
#include "kortex_driver/GetTwistSoftLimitation.h"
#include "kortex_driver/GetWrenchSoftLimitation.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class BaseServices
{
    public:
        BaseServices(ros::NodeHandle& n, Kinova::Api::Base::BaseClient* base, uint32_t device_id, uint32_t timeout_ms);

        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);
        bool CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res);
        bool UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res);
        bool ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res);
        bool DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res);
        bool ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res);
        bool ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res);
        bool ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res);
        bool CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res);
        bool UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res);
        bool ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res);
        bool DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res);
        bool ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res);
        bool DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res);
        bool DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res);
        bool PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res);
        bool PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res);
        bool StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res);
        bool PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res);
        bool ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res);
        bool CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res);
        bool UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res);
        bool ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res);
        bool DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res);
        bool ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res);
        bool CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res);
        bool ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res);
        bool ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res);
        bool CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res);
        bool ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res);
        bool ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res);
        bool CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res);
        bool ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res);
        bool ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res);
        bool DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res);
        bool UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res);
        bool ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res);
        bool ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res);
        bool PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res);
        bool StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res);
        bool ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res);
        bool GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res);
        bool SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res);
        bool SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res);
        bool IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res);
        bool GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res);
        bool GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res);
        bool AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res);
        bool DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res);
        bool GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res);
        bool ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res);
        bool DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res);
        bool GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res);
        bool Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request  &req, kortex_driver::Base_Unsubscribe::Response &res);
        bool OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res);
        void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif);
        bool OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res);
        void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif);
        bool OnNotificationControlModeTopic(kortex_driver::OnNotificationControlModeTopic::Request  &req, kortex_driver::OnNotificationControlModeTopic::Response &res);
        void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif);
        bool OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res);
        void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif);
        bool OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res);
        void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif);
        bool OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res);
        void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif);
        bool OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res);
        void cb_UserTopic(Kinova::Api::Base::UserNotification notif);
        bool OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res);
        void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif);
        bool OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res);
        void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif);
        bool OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res);
        void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif);
        bool PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res);
        bool PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res);
        bool PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res);
        bool Stop(kortex_driver::Stop::Request  &req, kortex_driver::Stop::Response &res);
        bool GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res);
        bool SendWrenchCommand(kortex_driver::SendWrenchCommand::Request  &req, kortex_driver::SendWrenchCommand::Response &res);
        bool SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request  &req, kortex_driver::SendWrenchJoystickCommand::Response &res);
        bool SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request  &req, kortex_driver::SendTwistJoystickCommand::Response &res);
        bool SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res);
        bool PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res);
        bool PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res);
        bool GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res);
        bool SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request  &req, kortex_driver::SendJointSpeedsCommand::Response &res);
        bool SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res);
        bool SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res);
        bool GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res);
        bool SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res);
        bool SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res);
        bool ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res);
        bool Base_ClearFaults(kortex_driver::Base_ClearFaults::Request  &req, kortex_driver::Base_ClearFaults::Response &res);
        bool Base_GetControlMode(kortex_driver::Base_GetControlMode::Request  &req, kortex_driver::Base_GetControlMode::Response &res);
        bool GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res);
        bool SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res);
        bool GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res);
        bool OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res);
        void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif);
        bool RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res);
        bool OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res);
        void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif);
        bool GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res);
        bool GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res);
        bool GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res);
        bool StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res);
        bool GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res);
        bool OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res);
        void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif);
        bool GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res);
        bool OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res);
        void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif);
        bool GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res);
        bool SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request  &req, kortex_driver::SetWifiCountryCode::Response &res);
        bool GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request  &req, kortex_driver::GetWifiCountryCode::Response &res);
        bool Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request  &req, kortex_driver::Base_SetCapSenseConfig::Response &res);
        bool Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request  &req, kortex_driver::Base_GetCapSenseConfig::Response &res);
        bool GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::GetAllJointsSpeedHardLimitation::Response &res);
        bool GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::GetAllJointsTorqueHardLimitation::Response &res);
        bool GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request  &req, kortex_driver::GetTwistHardLimitation::Response &res);
        bool GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request  &req, kortex_driver::GetWrenchHardLimitation::Response &res);
        bool SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::SendJointSpeedsJoystickCommand::Response &res);
        bool SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::SendSelectedJointSpeedJoystickCommand::Response &res);
        bool EnableBridge(kortex_driver::EnableBridge::Request  &req, kortex_driver::EnableBridge::Response &res);
        bool DisableBridge(kortex_driver::DisableBridge::Request  &req, kortex_driver::DisableBridge::Response &res);
        bool GetBridgeList(kortex_driver::GetBridgeList::Request  &req, kortex_driver::GetBridgeList::Response &res);
        bool GetBridgeConfig(kortex_driver::GetBridgeConfig::Request  &req, kortex_driver::GetBridgeConfig::Response &res);
        bool PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::PlayPreComputedJointTrajectory::Response &res);
        bool GetProductConfiguration(kortex_driver::GetProductConfiguration::Request  &req, kortex_driver::GetProductConfiguration::Response &res);
        bool UpdateDegreeOfFreedomConfiguration(kortex_driver::UpdateDegreeOfFreedomConfiguration::Request  &req, kortex_driver::UpdateDegreeOfFreedomConfiguration::Response &res);
        bool UpdateBaseTypeConfiguration(kortex_driver::UpdateBaseTypeConfiguration::Request  &req, kortex_driver::UpdateBaseTypeConfiguration::Response &res);
        bool UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::UpdateEndEffectorTypeConfiguration::Response &res);
        bool UpdateVisionModuleTypeConfiguration(kortex_driver::UpdateVisionModuleTypeConfiguration::Request  &req, kortex_driver::UpdateVisionModuleTypeConfiguration::Response &res);
        bool UpdateInterfaceModuleTypeConfiguration(kortex_driver::UpdateInterfaceModuleTypeConfiguration::Request  &req, kortex_driver::UpdateInterfaceModuleTypeConfiguration::Response &res);
        bool UpdateArmLateralityConfiguration(kortex_driver::UpdateArmLateralityConfiguration::Request  &req, kortex_driver::UpdateArmLateralityConfiguration::Response &res);
        bool UpdateWristTypeConfiguration(kortex_driver::UpdateWristTypeConfiguration::Request  &req, kortex_driver::UpdateWristTypeConfiguration::Response &res);
        bool RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::RestoreFactoryProductConfiguration::Response &res);
        bool GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request  &req, kortex_driver::GetTrajectoryErrorReport::Response &res);
        bool GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::GetAllJointsSpeedSoftLimitation::Response &res);
        bool GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::GetAllJointsTorqueSoftLimitation::Response &res);
        bool GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request  &req, kortex_driver::GetTwistSoftLimitation::Response &res);
        bool GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request  &req, kortex_driver::GetWrenchSoftLimitation::Response &res);

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::Base::BaseClient* m_base;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_ConfigurationChangeTopic;
        ros::Publisher m_pub_MappingInfoTopic;
        ros::Publisher m_pub_ControlModeTopic;
        ros::Publisher m_pub_OperatingModeTopic;
        ros::Publisher m_pub_SequenceInfoTopic;
        ros::Publisher m_pub_ProtectionZoneTopic;
        ros::Publisher m_pub_UserTopic;
        ros::Publisher m_pub_ControllerTopic;
        ros::Publisher m_pub_ActionTopic;
        ros::Publisher m_pub_RobotEventTopic;
        ros::Publisher m_pub_ServoingModeTopic;
        ros::Publisher m_pub_FactoryTopic;
        ros::Publisher m_pub_NetworkTopic;
        ros::Publisher m_pub_ArmStateTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceCreateUserProfile;
	ros::ServiceServer m_serviceUpdateUserProfile;
	ros::ServiceServer m_serviceReadUserProfile;
	ros::ServiceServer m_serviceDeleteUserProfile;
	ros::ServiceServer m_serviceReadAllUserProfiles;
	ros::ServiceServer m_serviceReadAllUsers;
	ros::ServiceServer m_serviceChangePassword;
	ros::ServiceServer m_serviceCreateSequence;
	ros::ServiceServer m_serviceUpdateSequence;
	ros::ServiceServer m_serviceReadSequence;
	ros::ServiceServer m_serviceDeleteSequence;
	ros::ServiceServer m_serviceReadAllSequences;
	ros::ServiceServer m_serviceDeleteSequenceTask;
	ros::ServiceServer m_serviceDeleteAllSequenceTasks;
	ros::ServiceServer m_servicePlaySequence;
	ros::ServiceServer m_servicePlayAdvancedSequence;
	ros::ServiceServer m_serviceStopSequence;
	ros::ServiceServer m_servicePauseSequence;
	ros::ServiceServer m_serviceResumeSequence;
	ros::ServiceServer m_serviceCreateProtectionZone;
	ros::ServiceServer m_serviceUpdateProtectionZone;
	ros::ServiceServer m_serviceReadProtectionZone;
	ros::ServiceServer m_serviceDeleteProtectionZone;
	ros::ServiceServer m_serviceReadAllProtectionZones;
	ros::ServiceServer m_serviceCreateMapping;
	ros::ServiceServer m_serviceReadMapping;
	ros::ServiceServer m_serviceReadAllMappings;
	ros::ServiceServer m_serviceCreateMap;
	ros::ServiceServer m_serviceReadAllMaps;
	ros::ServiceServer m_serviceActivateMap;
	ros::ServiceServer m_serviceCreateAction;
	ros::ServiceServer m_serviceReadAction;
	ros::ServiceServer m_serviceReadAllActions;
	ros::ServiceServer m_serviceDeleteAction;
	ros::ServiceServer m_serviceUpdateAction;
	ros::ServiceServer m_serviceExecuteActionFromReference;
	ros::ServiceServer m_serviceExecuteAction;
	ros::ServiceServer m_servicePauseAction;
	ros::ServiceServer m_serviceStopAction;
	ros::ServiceServer m_serviceResumeAction;
	ros::ServiceServer m_serviceGetIPv4Configuration;
	ros::ServiceServer m_serviceSetIPv4Configuration;
	ros::ServiceServer m_serviceSetCommunicationInterfaceEnable;
	ros::ServiceServer m_serviceIsCommunicationInterfaceEnable;
	ros::ServiceServer m_serviceGetAvailableWifi;
	ros::ServiceServer m_serviceGetWifiInformation;
	ros::ServiceServer m_serviceAddWifiConfiguration;
	ros::ServiceServer m_serviceDeleteWifiConfiguration;
	ros::ServiceServer m_serviceGetAllConfiguredWifis;
	ros::ServiceServer m_serviceConnectWifi;
	ros::ServiceServer m_serviceDisconnectWifi;
	ros::ServiceServer m_serviceGetConnectedWifiInformation;
	ros::ServiceServer m_serviceBase_Unsubscribe;
	ros::ServiceServer m_serviceOnNotificationConfigurationChangeTopic;
	ros::ServiceServer m_serviceOnNotificationMappingInfoTopic;
	ros::ServiceServer m_serviceOnNotificationControlModeTopic;
	ros::ServiceServer m_serviceOnNotificationOperatingModeTopic;
	ros::ServiceServer m_serviceOnNotificationSequenceInfoTopic;
	ros::ServiceServer m_serviceOnNotificationProtectionZoneTopic;
	ros::ServiceServer m_serviceOnNotificationUserTopic;
	ros::ServiceServer m_serviceOnNotificationControllerTopic;
	ros::ServiceServer m_serviceOnNotificationActionTopic;
	ros::ServiceServer m_serviceOnNotificationRobotEventTopic;
	ros::ServiceServer m_servicePlayCartesianTrajectory;
	ros::ServiceServer m_servicePlayCartesianTrajectoryPosition;
	ros::ServiceServer m_servicePlayCartesianTrajectoryOrientation;
	ros::ServiceServer m_serviceStop;
	ros::ServiceServer m_serviceGetMeasuredCartesianPose;
	ros::ServiceServer m_serviceSendWrenchCommand;
	ros::ServiceServer m_serviceSendWrenchJoystickCommand;
	ros::ServiceServer m_serviceSendTwistJoystickCommand;
	ros::ServiceServer m_serviceSendTwistCommand;
	ros::ServiceServer m_servicePlayJointTrajectory;
	ros::ServiceServer m_servicePlaySelectedJointTrajectory;
	ros::ServiceServer m_serviceGetMeasuredJointAngles;
	ros::ServiceServer m_serviceSendJointSpeedsCommand;
	ros::ServiceServer m_serviceSendSelectedJointSpeedCommand;
	ros::ServiceServer m_serviceSendGripperCommand;
	ros::ServiceServer m_serviceGetMeasuredGripperMovement;
	ros::ServiceServer m_serviceSetAdmittance;
	ros::ServiceServer m_serviceSetOperatingMode;
	ros::ServiceServer m_serviceApplyEmergencyStop;
	ros::ServiceServer m_serviceBase_ClearFaults;
	ros::ServiceServer m_serviceBase_GetControlMode;
	ros::ServiceServer m_serviceGetOperatingMode;
	ros::ServiceServer m_serviceSetServoingMode;
	ros::ServiceServer m_serviceGetServoingMode;
	ros::ServiceServer m_serviceOnNotificationServoingModeTopic;
	ros::ServiceServer m_serviceRestoreFactorySettings;
	ros::ServiceServer m_serviceOnNotificationFactoryTopic;
	ros::ServiceServer m_serviceGetAllConnectedControllers;
	ros::ServiceServer m_serviceGetControllerState;
	ros::ServiceServer m_serviceGetActuatorCount;
	ros::ServiceServer m_serviceStartWifiScan;
	ros::ServiceServer m_serviceGetConfiguredWifi;
	ros::ServiceServer m_serviceOnNotificationNetworkTopic;
	ros::ServiceServer m_serviceGetArmState;
	ros::ServiceServer m_serviceOnNotificationArmStateTopic;
	ros::ServiceServer m_serviceGetIPv4Information;
	ros::ServiceServer m_serviceSetWifiCountryCode;
	ros::ServiceServer m_serviceGetWifiCountryCode;
	ros::ServiceServer m_serviceBase_SetCapSenseConfig;
	ros::ServiceServer m_serviceBase_GetCapSenseConfig;
	ros::ServiceServer m_serviceGetAllJointsSpeedHardLimitation;
	ros::ServiceServer m_serviceGetAllJointsTorqueHardLimitation;
	ros::ServiceServer m_serviceGetTwistHardLimitation;
	ros::ServiceServer m_serviceGetWrenchHardLimitation;
	ros::ServiceServer m_serviceSendJointSpeedsJoystickCommand;
	ros::ServiceServer m_serviceSendSelectedJointSpeedJoystickCommand;
	ros::ServiceServer m_serviceEnableBridge;
	ros::ServiceServer m_serviceDisableBridge;
	ros::ServiceServer m_serviceGetBridgeList;
	ros::ServiceServer m_serviceGetBridgeConfig;
	ros::ServiceServer m_servicePlayPreComputedJointTrajectory;
	ros::ServiceServer m_serviceGetProductConfiguration;
	ros::ServiceServer m_serviceUpdateDegreeOfFreedomConfiguration;
	ros::ServiceServer m_serviceUpdateBaseTypeConfiguration;
	ros::ServiceServer m_serviceUpdateEndEffectorTypeConfiguration;
	ros::ServiceServer m_serviceUpdateVisionModuleTypeConfiguration;
	ros::ServiceServer m_serviceUpdateInterfaceModuleTypeConfiguration;
	ros::ServiceServer m_serviceUpdateArmLateralityConfiguration;
	ros::ServiceServer m_serviceUpdateWristTypeConfiguration;
	ros::ServiceServer m_serviceRestoreFactoryProductConfiguration;
	ros::ServiceServer m_serviceGetTrajectoryErrorReport;
	ros::ServiceServer m_serviceGetAllJointsSpeedSoftLimitation;
	ros::ServiceServer m_serviceGetAllJointsTorqueSoftLimitation;
	ros::ServiceServer m_serviceGetTwistSoftLimitation;
	ros::ServiceServer m_serviceGetWrenchSoftLimitation;
};
#endif
