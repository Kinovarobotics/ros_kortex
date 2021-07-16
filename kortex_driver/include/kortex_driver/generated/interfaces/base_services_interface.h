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
 
#ifndef _KORTEX_BASE_SERVICES_INTERFACE_H_
#define _KORTEX_BASE_SERVICES_INTERFACE_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
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
#include "kortex_driver/UpdateMapping.h"
#include "kortex_driver/DeleteMapping.h"
#include "kortex_driver/ReadAllMappings.h"
#include "kortex_driver/CreateMap.h"
#include "kortex_driver/ReadMap.h"
#include "kortex_driver/UpdateMap.h"
#include "kortex_driver/DeleteMap.h"
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
#include "kortex_driver/Base_OnNotificationControlModeTopic.h"
#include "kortex_driver/Base_ControlModeNotification.h"
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
#include "kortex_driver/UpdateEndEffectorTypeConfiguration.h"
#include "kortex_driver/RestoreFactoryProductConfiguration.h"
#include "kortex_driver/GetTrajectoryErrorReport.h"
#include "kortex_driver/GetAllJointsSpeedSoftLimitation.h"
#include "kortex_driver/GetAllJointsTorqueSoftLimitation.h"
#include "kortex_driver/GetTwistSoftLimitation.h"
#include "kortex_driver/GetWrenchSoftLimitation.h"
#include "kortex_driver/SetControllerConfigurationMode.h"
#include "kortex_driver/GetControllerConfigurationMode.h"
#include "kortex_driver/StartTeaching.h"
#include "kortex_driver/StopTeaching.h"
#include "kortex_driver/AddSequenceTasks.h"
#include "kortex_driver/UpdateSequenceTask.h"
#include "kortex_driver/SwapSequenceTasks.h"
#include "kortex_driver/ReadSequenceTask.h"
#include "kortex_driver/ReadAllSequenceTasks.h"
#include "kortex_driver/DeleteSequenceTask.h"
#include "kortex_driver/DeleteAllSequenceTasks.h"
#include "kortex_driver/TakeSnapshot.h"
#include "kortex_driver/GetFirmwareBundleVersions.h"
#include "kortex_driver/ExecuteWaypointTrajectory.h"
#include "kortex_driver/MoveSequenceTask.h"
#include "kortex_driver/DuplicateMapping.h"
#include "kortex_driver/DuplicateMap.h"
#include "kortex_driver/SetControllerConfiguration.h"
#include "kortex_driver/GetControllerConfiguration.h"
#include "kortex_driver/GetAllControllerConfigurations.h"
#include "kortex_driver/ComputeForwardKinematics.h"
#include "kortex_driver/ComputeInverseKinematics.h"
#include "kortex_driver/ValidateWaypointList.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class IBaseServices
{
    public:
        IBaseServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) = 0;
        virtual bool CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res) = 0;
        virtual bool UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res) = 0;
        virtual bool ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res) = 0;
        virtual bool DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res) = 0;
        virtual bool ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res) = 0;
        virtual bool ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res) = 0;
        virtual bool ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res) = 0;
        virtual bool CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res) = 0;
        virtual bool UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res) = 0;
        virtual bool ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res) = 0;
        virtual bool DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res) = 0;
        virtual bool ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res) = 0;
        virtual bool PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res) = 0;
        virtual bool PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res) = 0;
        virtual bool StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res) = 0;
        virtual bool PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res) = 0;
        virtual bool ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res) = 0;
        virtual bool CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res) = 0;
        virtual bool UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res) = 0;
        virtual bool ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res) = 0;
        virtual bool DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res) = 0;
        virtual bool ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res) = 0;
        virtual bool CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res) = 0;
        virtual bool ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res) = 0;
        virtual bool UpdateMapping(kortex_driver::UpdateMapping::Request  &req, kortex_driver::UpdateMapping::Response &res) = 0;
        virtual bool DeleteMapping(kortex_driver::DeleteMapping::Request  &req, kortex_driver::DeleteMapping::Response &res) = 0;
        virtual bool ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res) = 0;
        virtual bool CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res) = 0;
        virtual bool ReadMap(kortex_driver::ReadMap::Request  &req, kortex_driver::ReadMap::Response &res) = 0;
        virtual bool UpdateMap(kortex_driver::UpdateMap::Request  &req, kortex_driver::UpdateMap::Response &res) = 0;
        virtual bool DeleteMap(kortex_driver::DeleteMap::Request  &req, kortex_driver::DeleteMap::Response &res) = 0;
        virtual bool ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res) = 0;
        virtual bool ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res) = 0;
        virtual bool CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res) = 0;
        virtual bool ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res) = 0;
        virtual bool ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res) = 0;
        virtual bool DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res) = 0;
        virtual bool UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res) = 0;
        virtual bool ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res) = 0;
        virtual bool ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res) = 0;
        virtual bool PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res) = 0;
        virtual bool StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res) = 0;
        virtual bool ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res) = 0;
        virtual bool GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res) = 0;
        virtual bool SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res) = 0;
        virtual bool SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res) = 0;
        virtual bool IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res) = 0;
        virtual bool GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res) = 0;
        virtual bool GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res) = 0;
        virtual bool AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res) = 0;
        virtual bool DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res) = 0;
        virtual bool GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res) = 0;
        virtual bool ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res) = 0;
        virtual bool DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res) = 0;
        virtual bool GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res) = 0;
        virtual bool Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request  &req, kortex_driver::Base_Unsubscribe::Response &res) = 0;
        virtual bool OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res) = 0;
        virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) = 0;
        virtual bool OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res) = 0;
        virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) = 0;
        virtual bool Base_OnNotificationControlModeTopic(kortex_driver::Base_OnNotificationControlModeTopic::Request  &req, kortex_driver::Base_OnNotificationControlModeTopic::Response &res) = 0;
        virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) = 0;
        virtual bool OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res) = 0;
        virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) = 0;
        virtual bool OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res) = 0;
        virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) = 0;
        virtual bool OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res) = 0;
        virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) = 0;
        virtual bool OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res) = 0;
        virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) = 0;
        virtual bool OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res) = 0;
        virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) = 0;
        virtual bool OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res) = 0;
        virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) = 0;
        virtual bool OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res) = 0;
        virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) = 0;
        virtual bool PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res) = 0;
        virtual bool PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res) = 0;
        virtual bool PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res) = 0;
        virtual bool Stop(kortex_driver::Stop::Request  &req, kortex_driver::Stop::Response &res) = 0;
        virtual bool GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res) = 0;
        virtual bool SendWrenchCommand(kortex_driver::SendWrenchCommand::Request  &req, kortex_driver::SendWrenchCommand::Response &res) = 0;
        virtual bool SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request  &req, kortex_driver::SendWrenchJoystickCommand::Response &res) = 0;
        virtual bool SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request  &req, kortex_driver::SendTwistJoystickCommand::Response &res) = 0;
        virtual bool SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res) = 0;
        virtual bool PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res) = 0;
        virtual bool PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res) = 0;
        virtual bool GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res) = 0;
        virtual bool SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request  &req, kortex_driver::SendJointSpeedsCommand::Response &res) = 0;
        virtual bool SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res) = 0;
        virtual bool SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res) = 0;
        virtual bool GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res) = 0;
        virtual bool SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res) = 0;
        virtual bool SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res) = 0;
        virtual bool ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res) = 0;
        virtual bool Base_ClearFaults(kortex_driver::Base_ClearFaults::Request  &req, kortex_driver::Base_ClearFaults::Response &res) = 0;
        virtual bool Base_GetControlMode(kortex_driver::Base_GetControlMode::Request  &req, kortex_driver::Base_GetControlMode::Response &res) = 0;
        virtual bool GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res) = 0;
        virtual bool SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res) = 0;
        virtual bool GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res) = 0;
        virtual bool OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res) = 0;
        virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) = 0;
        virtual bool RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res) = 0;
        virtual bool OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res) = 0;
        virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) = 0;
        virtual bool GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res) = 0;
        virtual bool GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res) = 0;
        virtual bool GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res) = 0;
        virtual bool StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res) = 0;
        virtual bool GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res) = 0;
        virtual bool OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res) = 0;
        virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) = 0;
        virtual bool GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res) = 0;
        virtual bool OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res) = 0;
        virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) = 0;
        virtual bool GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res) = 0;
        virtual bool SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request  &req, kortex_driver::SetWifiCountryCode::Response &res) = 0;
        virtual bool GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request  &req, kortex_driver::GetWifiCountryCode::Response &res) = 0;
        virtual bool Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request  &req, kortex_driver::Base_SetCapSenseConfig::Response &res) = 0;
        virtual bool Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request  &req, kortex_driver::Base_GetCapSenseConfig::Response &res) = 0;
        virtual bool GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::GetAllJointsSpeedHardLimitation::Response &res) = 0;
        virtual bool GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::GetAllJointsTorqueHardLimitation::Response &res) = 0;
        virtual bool GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request  &req, kortex_driver::GetTwistHardLimitation::Response &res) = 0;
        virtual bool GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request  &req, kortex_driver::GetWrenchHardLimitation::Response &res) = 0;
        virtual bool SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::SendJointSpeedsJoystickCommand::Response &res) = 0;
        virtual bool SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::SendSelectedJointSpeedJoystickCommand::Response &res) = 0;
        virtual bool EnableBridge(kortex_driver::EnableBridge::Request  &req, kortex_driver::EnableBridge::Response &res) = 0;
        virtual bool DisableBridge(kortex_driver::DisableBridge::Request  &req, kortex_driver::DisableBridge::Response &res) = 0;
        virtual bool GetBridgeList(kortex_driver::GetBridgeList::Request  &req, kortex_driver::GetBridgeList::Response &res) = 0;
        virtual bool GetBridgeConfig(kortex_driver::GetBridgeConfig::Request  &req, kortex_driver::GetBridgeConfig::Response &res) = 0;
        virtual bool PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::PlayPreComputedJointTrajectory::Response &res) = 0;
        virtual bool GetProductConfiguration(kortex_driver::GetProductConfiguration::Request  &req, kortex_driver::GetProductConfiguration::Response &res) = 0;
        virtual bool UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::UpdateEndEffectorTypeConfiguration::Response &res) = 0;
        virtual bool RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::RestoreFactoryProductConfiguration::Response &res) = 0;
        virtual bool GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request  &req, kortex_driver::GetTrajectoryErrorReport::Response &res) = 0;
        virtual bool GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::GetAllJointsSpeedSoftLimitation::Response &res) = 0;
        virtual bool GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::GetAllJointsTorqueSoftLimitation::Response &res) = 0;
        virtual bool GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request  &req, kortex_driver::GetTwistSoftLimitation::Response &res) = 0;
        virtual bool GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request  &req, kortex_driver::GetWrenchSoftLimitation::Response &res) = 0;
        virtual bool SetControllerConfigurationMode(kortex_driver::SetControllerConfigurationMode::Request  &req, kortex_driver::SetControllerConfigurationMode::Response &res) = 0;
        virtual bool GetControllerConfigurationMode(kortex_driver::GetControllerConfigurationMode::Request  &req, kortex_driver::GetControllerConfigurationMode::Response &res) = 0;
        virtual bool StartTeaching(kortex_driver::StartTeaching::Request  &req, kortex_driver::StartTeaching::Response &res) = 0;
        virtual bool StopTeaching(kortex_driver::StopTeaching::Request  &req, kortex_driver::StopTeaching::Response &res) = 0;
        virtual bool AddSequenceTasks(kortex_driver::AddSequenceTasks::Request  &req, kortex_driver::AddSequenceTasks::Response &res) = 0;
        virtual bool UpdateSequenceTask(kortex_driver::UpdateSequenceTask::Request  &req, kortex_driver::UpdateSequenceTask::Response &res) = 0;
        virtual bool SwapSequenceTasks(kortex_driver::SwapSequenceTasks::Request  &req, kortex_driver::SwapSequenceTasks::Response &res) = 0;
        virtual bool ReadSequenceTask(kortex_driver::ReadSequenceTask::Request  &req, kortex_driver::ReadSequenceTask::Response &res) = 0;
        virtual bool ReadAllSequenceTasks(kortex_driver::ReadAllSequenceTasks::Request  &req, kortex_driver::ReadAllSequenceTasks::Response &res) = 0;
        virtual bool DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res) = 0;
        virtual bool DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res) = 0;
        virtual bool TakeSnapshot(kortex_driver::TakeSnapshot::Request  &req, kortex_driver::TakeSnapshot::Response &res) = 0;
        virtual bool GetFirmwareBundleVersions(kortex_driver::GetFirmwareBundleVersions::Request  &req, kortex_driver::GetFirmwareBundleVersions::Response &res) = 0;
        virtual bool ExecuteWaypointTrajectory(kortex_driver::ExecuteWaypointTrajectory::Request  &req, kortex_driver::ExecuteWaypointTrajectory::Response &res) = 0;
        virtual bool MoveSequenceTask(kortex_driver::MoveSequenceTask::Request  &req, kortex_driver::MoveSequenceTask::Response &res) = 0;
        virtual bool DuplicateMapping(kortex_driver::DuplicateMapping::Request  &req, kortex_driver::DuplicateMapping::Response &res) = 0;
        virtual bool DuplicateMap(kortex_driver::DuplicateMap::Request  &req, kortex_driver::DuplicateMap::Response &res) = 0;
        virtual bool SetControllerConfiguration(kortex_driver::SetControllerConfiguration::Request  &req, kortex_driver::SetControllerConfiguration::Response &res) = 0;
        virtual bool GetControllerConfiguration(kortex_driver::GetControllerConfiguration::Request  &req, kortex_driver::GetControllerConfiguration::Response &res) = 0;
        virtual bool GetAllControllerConfigurations(kortex_driver::GetAllControllerConfigurations::Request  &req, kortex_driver::GetAllControllerConfigurations::Response &res) = 0;
        virtual bool ComputeForwardKinematics(kortex_driver::ComputeForwardKinematics::Request  &req, kortex_driver::ComputeForwardKinematics::Response &res) = 0;
        virtual bool ComputeInverseKinematics(kortex_driver::ComputeInverseKinematics::Request  &req, kortex_driver::ComputeInverseKinematics::Response &res) = 0;
        virtual bool ValidateWaypointList(kortex_driver::ValidateWaypointList::Request  &req, kortex_driver::ValidateWaypointList::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_ConfigurationChangeTopic;
        bool m_is_activated_ConfigurationChangeTopic;
        ros::Publisher m_pub_MappingInfoTopic;
        bool m_is_activated_MappingInfoTopic;
        ros::Publisher m_pub_ControlModeTopic;
        bool m_is_activated_ControlModeTopic;
        ros::Publisher m_pub_OperatingModeTopic;
        bool m_is_activated_OperatingModeTopic;
        ros::Publisher m_pub_SequenceInfoTopic;
        bool m_is_activated_SequenceInfoTopic;
        ros::Publisher m_pub_ProtectionZoneTopic;
        bool m_is_activated_ProtectionZoneTopic;
        ros::Publisher m_pub_UserTopic;
        bool m_is_activated_UserTopic;
        ros::Publisher m_pub_ControllerTopic;
        bool m_is_activated_ControllerTopic;
        ros::Publisher m_pub_ActionTopic;
        bool m_is_activated_ActionTopic;
        ros::Publisher m_pub_RobotEventTopic;
        bool m_is_activated_RobotEventTopic;
        ros::Publisher m_pub_ServoingModeTopic;
        bool m_is_activated_ServoingModeTopic;
        ros::Publisher m_pub_FactoryTopic;
        bool m_is_activated_FactoryTopic;
        ros::Publisher m_pub_NetworkTopic;
        bool m_is_activated_NetworkTopic;
        ros::Publisher m_pub_ArmStateTopic;
        bool m_is_activated_ArmStateTopic;

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
	ros::ServiceServer m_serviceUpdateMapping;
	ros::ServiceServer m_serviceDeleteMapping;
	ros::ServiceServer m_serviceReadAllMappings;
	ros::ServiceServer m_serviceCreateMap;
	ros::ServiceServer m_serviceReadMap;
	ros::ServiceServer m_serviceUpdateMap;
	ros::ServiceServer m_serviceDeleteMap;
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
	ros::ServiceServer m_serviceBase_OnNotificationControlModeTopic;
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
	ros::ServiceServer m_serviceUpdateEndEffectorTypeConfiguration;
	ros::ServiceServer m_serviceRestoreFactoryProductConfiguration;
	ros::ServiceServer m_serviceGetTrajectoryErrorReport;
	ros::ServiceServer m_serviceGetAllJointsSpeedSoftLimitation;
	ros::ServiceServer m_serviceGetAllJointsTorqueSoftLimitation;
	ros::ServiceServer m_serviceGetTwistSoftLimitation;
	ros::ServiceServer m_serviceGetWrenchSoftLimitation;
	ros::ServiceServer m_serviceSetControllerConfigurationMode;
	ros::ServiceServer m_serviceGetControllerConfigurationMode;
	ros::ServiceServer m_serviceStartTeaching;
	ros::ServiceServer m_serviceStopTeaching;
	ros::ServiceServer m_serviceAddSequenceTasks;
	ros::ServiceServer m_serviceUpdateSequenceTask;
	ros::ServiceServer m_serviceSwapSequenceTasks;
	ros::ServiceServer m_serviceReadSequenceTask;
	ros::ServiceServer m_serviceReadAllSequenceTasks;
	ros::ServiceServer m_serviceDeleteSequenceTask;
	ros::ServiceServer m_serviceDeleteAllSequenceTasks;
	ros::ServiceServer m_serviceTakeSnapshot;
	ros::ServiceServer m_serviceGetFirmwareBundleVersions;
	ros::ServiceServer m_serviceExecuteWaypointTrajectory;
	ros::ServiceServer m_serviceMoveSequenceTask;
	ros::ServiceServer m_serviceDuplicateMapping;
	ros::ServiceServer m_serviceDuplicateMap;
	ros::ServiceServer m_serviceSetControllerConfiguration;
	ros::ServiceServer m_serviceGetControllerConfiguration;
	ros::ServiceServer m_serviceGetAllControllerConfigurations;
	ros::ServiceServer m_serviceComputeForwardKinematics;
	ros::ServiceServer m_serviceComputeInverseKinematics;
	ros::ServiceServer m_serviceValidateWaypointList;
};
#endif
