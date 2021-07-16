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
 
#ifndef _KORTEX_BASE_ROBOT_SERVICES_H_
#define _KORTEX_BASE_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/base_services_interface.h"

#include <Base.pb.h>
#include <BaseClientRpc.h>

using namespace std;

class BaseRobotServices : public IBaseServices
{
    public:
        BaseRobotServices(ros::NodeHandle& node_handle, Kinova::Api::Base::BaseClient* base, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        virtual bool CreateUserProfile(kortex_driver::CreateUserProfile::Request  &req, kortex_driver::CreateUserProfile::Response &res) override;
        virtual bool UpdateUserProfile(kortex_driver::UpdateUserProfile::Request  &req, kortex_driver::UpdateUserProfile::Response &res) override;
        virtual bool ReadUserProfile(kortex_driver::ReadUserProfile::Request  &req, kortex_driver::ReadUserProfile::Response &res) override;
        virtual bool DeleteUserProfile(kortex_driver::DeleteUserProfile::Request  &req, kortex_driver::DeleteUserProfile::Response &res) override;
        virtual bool ReadAllUserProfiles(kortex_driver::ReadAllUserProfiles::Request  &req, kortex_driver::ReadAllUserProfiles::Response &res) override;
        virtual bool ReadAllUsers(kortex_driver::ReadAllUsers::Request  &req, kortex_driver::ReadAllUsers::Response &res) override;
        virtual bool ChangePassword(kortex_driver::ChangePassword::Request  &req, kortex_driver::ChangePassword::Response &res) override;
        virtual bool CreateSequence(kortex_driver::CreateSequence::Request  &req, kortex_driver::CreateSequence::Response &res) override;
        virtual bool UpdateSequence(kortex_driver::UpdateSequence::Request  &req, kortex_driver::UpdateSequence::Response &res) override;
        virtual bool ReadSequence(kortex_driver::ReadSequence::Request  &req, kortex_driver::ReadSequence::Response &res) override;
        virtual bool DeleteSequence(kortex_driver::DeleteSequence::Request  &req, kortex_driver::DeleteSequence::Response &res) override;
        virtual bool ReadAllSequences(kortex_driver::ReadAllSequences::Request  &req, kortex_driver::ReadAllSequences::Response &res) override;
        virtual bool PlaySequence(kortex_driver::PlaySequence::Request  &req, kortex_driver::PlaySequence::Response &res) override;
        virtual bool PlayAdvancedSequence(kortex_driver::PlayAdvancedSequence::Request  &req, kortex_driver::PlayAdvancedSequence::Response &res) override;
        virtual bool StopSequence(kortex_driver::StopSequence::Request  &req, kortex_driver::StopSequence::Response &res) override;
        virtual bool PauseSequence(kortex_driver::PauseSequence::Request  &req, kortex_driver::PauseSequence::Response &res) override;
        virtual bool ResumeSequence(kortex_driver::ResumeSequence::Request  &req, kortex_driver::ResumeSequence::Response &res) override;
        virtual bool CreateProtectionZone(kortex_driver::CreateProtectionZone::Request  &req, kortex_driver::CreateProtectionZone::Response &res) override;
        virtual bool UpdateProtectionZone(kortex_driver::UpdateProtectionZone::Request  &req, kortex_driver::UpdateProtectionZone::Response &res) override;
        virtual bool ReadProtectionZone(kortex_driver::ReadProtectionZone::Request  &req, kortex_driver::ReadProtectionZone::Response &res) override;
        virtual bool DeleteProtectionZone(kortex_driver::DeleteProtectionZone::Request  &req, kortex_driver::DeleteProtectionZone::Response &res) override;
        virtual bool ReadAllProtectionZones(kortex_driver::ReadAllProtectionZones::Request  &req, kortex_driver::ReadAllProtectionZones::Response &res) override;
        virtual bool CreateMapping(kortex_driver::CreateMapping::Request  &req, kortex_driver::CreateMapping::Response &res) override;
        virtual bool ReadMapping(kortex_driver::ReadMapping::Request  &req, kortex_driver::ReadMapping::Response &res) override;
        virtual bool UpdateMapping(kortex_driver::UpdateMapping::Request  &req, kortex_driver::UpdateMapping::Response &res) override;
        virtual bool DeleteMapping(kortex_driver::DeleteMapping::Request  &req, kortex_driver::DeleteMapping::Response &res) override;
        virtual bool ReadAllMappings(kortex_driver::ReadAllMappings::Request  &req, kortex_driver::ReadAllMappings::Response &res) override;
        virtual bool CreateMap(kortex_driver::CreateMap::Request  &req, kortex_driver::CreateMap::Response &res) override;
        virtual bool ReadMap(kortex_driver::ReadMap::Request  &req, kortex_driver::ReadMap::Response &res) override;
        virtual bool UpdateMap(kortex_driver::UpdateMap::Request  &req, kortex_driver::UpdateMap::Response &res) override;
        virtual bool DeleteMap(kortex_driver::DeleteMap::Request  &req, kortex_driver::DeleteMap::Response &res) override;
        virtual bool ReadAllMaps(kortex_driver::ReadAllMaps::Request  &req, kortex_driver::ReadAllMaps::Response &res) override;
        virtual bool ActivateMap(kortex_driver::ActivateMap::Request  &req, kortex_driver::ActivateMap::Response &res) override;
        virtual bool CreateAction(kortex_driver::CreateAction::Request  &req, kortex_driver::CreateAction::Response &res) override;
        virtual bool ReadAction(kortex_driver::ReadAction::Request  &req, kortex_driver::ReadAction::Response &res) override;
        virtual bool ReadAllActions(kortex_driver::ReadAllActions::Request  &req, kortex_driver::ReadAllActions::Response &res) override;
        virtual bool DeleteAction(kortex_driver::DeleteAction::Request  &req, kortex_driver::DeleteAction::Response &res) override;
        virtual bool UpdateAction(kortex_driver::UpdateAction::Request  &req, kortex_driver::UpdateAction::Response &res) override;
        virtual bool ExecuteActionFromReference(kortex_driver::ExecuteActionFromReference::Request  &req, kortex_driver::ExecuteActionFromReference::Response &res) override;
        virtual bool ExecuteAction(kortex_driver::ExecuteAction::Request  &req, kortex_driver::ExecuteAction::Response &res) override;
        virtual bool PauseAction(kortex_driver::PauseAction::Request  &req, kortex_driver::PauseAction::Response &res) override;
        virtual bool StopAction(kortex_driver::StopAction::Request  &req, kortex_driver::StopAction::Response &res) override;
        virtual bool ResumeAction(kortex_driver::ResumeAction::Request  &req, kortex_driver::ResumeAction::Response &res) override;
        virtual bool GetIPv4Configuration(kortex_driver::GetIPv4Configuration::Request  &req, kortex_driver::GetIPv4Configuration::Response &res) override;
        virtual bool SetIPv4Configuration(kortex_driver::SetIPv4Configuration::Request  &req, kortex_driver::SetIPv4Configuration::Response &res) override;
        virtual bool SetCommunicationInterfaceEnable(kortex_driver::SetCommunicationInterfaceEnable::Request  &req, kortex_driver::SetCommunicationInterfaceEnable::Response &res) override;
        virtual bool IsCommunicationInterfaceEnable(kortex_driver::IsCommunicationInterfaceEnable::Request  &req, kortex_driver::IsCommunicationInterfaceEnable::Response &res) override;
        virtual bool GetAvailableWifi(kortex_driver::GetAvailableWifi::Request  &req, kortex_driver::GetAvailableWifi::Response &res) override;
        virtual bool GetWifiInformation(kortex_driver::GetWifiInformation::Request  &req, kortex_driver::GetWifiInformation::Response &res) override;
        virtual bool AddWifiConfiguration(kortex_driver::AddWifiConfiguration::Request  &req, kortex_driver::AddWifiConfiguration::Response &res) override;
        virtual bool DeleteWifiConfiguration(kortex_driver::DeleteWifiConfiguration::Request  &req, kortex_driver::DeleteWifiConfiguration::Response &res) override;
        virtual bool GetAllConfiguredWifis(kortex_driver::GetAllConfiguredWifis::Request  &req, kortex_driver::GetAllConfiguredWifis::Response &res) override;
        virtual bool ConnectWifi(kortex_driver::ConnectWifi::Request  &req, kortex_driver::ConnectWifi::Response &res) override;
        virtual bool DisconnectWifi(kortex_driver::DisconnectWifi::Request  &req, kortex_driver::DisconnectWifi::Response &res) override;
        virtual bool GetConnectedWifiInformation(kortex_driver::GetConnectedWifiInformation::Request  &req, kortex_driver::GetConnectedWifiInformation::Response &res) override;
        virtual bool Base_Unsubscribe(kortex_driver::Base_Unsubscribe::Request  &req, kortex_driver::Base_Unsubscribe::Response &res) override;
        virtual bool OnNotificationConfigurationChangeTopic(kortex_driver::OnNotificationConfigurationChangeTopic::Request  &req, kortex_driver::OnNotificationConfigurationChangeTopic::Response &res) override;
        virtual void cb_ConfigurationChangeTopic(Kinova::Api::Base::ConfigurationChangeNotification notif) override;
        virtual bool OnNotificationMappingInfoTopic(kortex_driver::OnNotificationMappingInfoTopic::Request  &req, kortex_driver::OnNotificationMappingInfoTopic::Response &res) override;
        virtual void cb_MappingInfoTopic(Kinova::Api::Base::MappingInfoNotification notif) override;
        virtual bool Base_OnNotificationControlModeTopic(kortex_driver::Base_OnNotificationControlModeTopic::Request  &req, kortex_driver::Base_OnNotificationControlModeTopic::Response &res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::Base::ControlModeNotification notif) override;
        virtual bool OnNotificationOperatingModeTopic(kortex_driver::OnNotificationOperatingModeTopic::Request  &req, kortex_driver::OnNotificationOperatingModeTopic::Response &res) override;
        virtual void cb_OperatingModeTopic(Kinova::Api::Base::OperatingModeNotification notif) override;
        virtual bool OnNotificationSequenceInfoTopic(kortex_driver::OnNotificationSequenceInfoTopic::Request  &req, kortex_driver::OnNotificationSequenceInfoTopic::Response &res) override;
        virtual void cb_SequenceInfoTopic(Kinova::Api::Base::SequenceInfoNotification notif) override;
        virtual bool OnNotificationProtectionZoneTopic(kortex_driver::OnNotificationProtectionZoneTopic::Request  &req, kortex_driver::OnNotificationProtectionZoneTopic::Response &res) override;
        virtual void cb_ProtectionZoneTopic(Kinova::Api::Base::ProtectionZoneNotification notif) override;
        virtual bool OnNotificationUserTopic(kortex_driver::OnNotificationUserTopic::Request  &req, kortex_driver::OnNotificationUserTopic::Response &res) override;
        virtual void cb_UserTopic(Kinova::Api::Base::UserNotification notif) override;
        virtual bool OnNotificationControllerTopic(kortex_driver::OnNotificationControllerTopic::Request  &req, kortex_driver::OnNotificationControllerTopic::Response &res) override;
        virtual void cb_ControllerTopic(Kinova::Api::Base::ControllerNotification notif) override;
        virtual bool OnNotificationActionTopic(kortex_driver::OnNotificationActionTopic::Request  &req, kortex_driver::OnNotificationActionTopic::Response &res) override;
        virtual void cb_ActionTopic(Kinova::Api::Base::ActionNotification notif) override;
        virtual bool OnNotificationRobotEventTopic(kortex_driver::OnNotificationRobotEventTopic::Request  &req, kortex_driver::OnNotificationRobotEventTopic::Response &res) override;
        virtual void cb_RobotEventTopic(Kinova::Api::Base::RobotEventNotification notif) override;
        virtual bool PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res) override;
        virtual bool PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res) override;
        virtual bool PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res) override;
        virtual bool Stop(kortex_driver::Stop::Request  &req, kortex_driver::Stop::Response &res) override;
        virtual bool GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res) override;
        virtual bool SendWrenchCommand(kortex_driver::SendWrenchCommand::Request  &req, kortex_driver::SendWrenchCommand::Response &res) override;
        virtual bool SendWrenchJoystickCommand(kortex_driver::SendWrenchJoystickCommand::Request  &req, kortex_driver::SendWrenchJoystickCommand::Response &res) override;
        virtual bool SendTwistJoystickCommand(kortex_driver::SendTwistJoystickCommand::Request  &req, kortex_driver::SendTwistJoystickCommand::Response &res) override;
        virtual bool SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res) override;
        virtual bool PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res) override;
        virtual bool PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res) override;
        virtual bool GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res) override;
        virtual bool SendJointSpeedsCommand(kortex_driver::SendJointSpeedsCommand::Request  &req, kortex_driver::SendJointSpeedsCommand::Response &res) override;
        virtual bool SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res) override;
        virtual bool SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res) override;
        virtual bool GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res) override;
        virtual bool SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res) override;
        virtual bool SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res) override;
        virtual bool ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res) override;
        virtual bool Base_ClearFaults(kortex_driver::Base_ClearFaults::Request  &req, kortex_driver::Base_ClearFaults::Response &res) override;
        virtual bool Base_GetControlMode(kortex_driver::Base_GetControlMode::Request  &req, kortex_driver::Base_GetControlMode::Response &res) override;
        virtual bool GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res) override;
        virtual bool SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res) override;
        virtual bool GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res) override;
        virtual bool OnNotificationServoingModeTopic(kortex_driver::OnNotificationServoingModeTopic::Request  &req, kortex_driver::OnNotificationServoingModeTopic::Response &res) override;
        virtual void cb_ServoingModeTopic(Kinova::Api::Base::ServoingModeNotification notif) override;
        virtual bool RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res) override;
        virtual bool OnNotificationFactoryTopic(kortex_driver::OnNotificationFactoryTopic::Request  &req, kortex_driver::OnNotificationFactoryTopic::Response &res) override;
        virtual void cb_FactoryTopic(Kinova::Api::Base::FactoryNotification notif) override;
        virtual bool GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res) override;
        virtual bool GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res) override;
        virtual bool GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res) override;
        virtual bool StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res) override;
        virtual bool GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res) override;
        virtual bool OnNotificationNetworkTopic(kortex_driver::OnNotificationNetworkTopic::Request  &req, kortex_driver::OnNotificationNetworkTopic::Response &res) override;
        virtual void cb_NetworkTopic(Kinova::Api::Base::NetworkNotification notif) override;
        virtual bool GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res) override;
        virtual bool OnNotificationArmStateTopic(kortex_driver::OnNotificationArmStateTopic::Request  &req, kortex_driver::OnNotificationArmStateTopic::Response &res) override;
        virtual void cb_ArmStateTopic(Kinova::Api::Base::ArmStateNotification notif) override;
        virtual bool GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res) override;
        virtual bool SetWifiCountryCode(kortex_driver::SetWifiCountryCode::Request  &req, kortex_driver::SetWifiCountryCode::Response &res) override;
        virtual bool GetWifiCountryCode(kortex_driver::GetWifiCountryCode::Request  &req, kortex_driver::GetWifiCountryCode::Response &res) override;
        virtual bool Base_SetCapSenseConfig(kortex_driver::Base_SetCapSenseConfig::Request  &req, kortex_driver::Base_SetCapSenseConfig::Response &res) override;
        virtual bool Base_GetCapSenseConfig(kortex_driver::Base_GetCapSenseConfig::Request  &req, kortex_driver::Base_GetCapSenseConfig::Response &res) override;
        virtual bool GetAllJointsSpeedHardLimitation(kortex_driver::GetAllJointsSpeedHardLimitation::Request  &req, kortex_driver::GetAllJointsSpeedHardLimitation::Response &res) override;
        virtual bool GetAllJointsTorqueHardLimitation(kortex_driver::GetAllJointsTorqueHardLimitation::Request  &req, kortex_driver::GetAllJointsTorqueHardLimitation::Response &res) override;
        virtual bool GetTwistHardLimitation(kortex_driver::GetTwistHardLimitation::Request  &req, kortex_driver::GetTwistHardLimitation::Response &res) override;
        virtual bool GetWrenchHardLimitation(kortex_driver::GetWrenchHardLimitation::Request  &req, kortex_driver::GetWrenchHardLimitation::Response &res) override;
        virtual bool SendJointSpeedsJoystickCommand(kortex_driver::SendJointSpeedsJoystickCommand::Request  &req, kortex_driver::SendJointSpeedsJoystickCommand::Response &res) override;
        virtual bool SendSelectedJointSpeedJoystickCommand(kortex_driver::SendSelectedJointSpeedJoystickCommand::Request  &req, kortex_driver::SendSelectedJointSpeedJoystickCommand::Response &res) override;
        virtual bool EnableBridge(kortex_driver::EnableBridge::Request  &req, kortex_driver::EnableBridge::Response &res) override;
        virtual bool DisableBridge(kortex_driver::DisableBridge::Request  &req, kortex_driver::DisableBridge::Response &res) override;
        virtual bool GetBridgeList(kortex_driver::GetBridgeList::Request  &req, kortex_driver::GetBridgeList::Response &res) override;
        virtual bool GetBridgeConfig(kortex_driver::GetBridgeConfig::Request  &req, kortex_driver::GetBridgeConfig::Response &res) override;
        virtual bool PlayPreComputedJointTrajectory(kortex_driver::PlayPreComputedJointTrajectory::Request  &req, kortex_driver::PlayPreComputedJointTrajectory::Response &res) override;
        virtual bool GetProductConfiguration(kortex_driver::GetProductConfiguration::Request  &req, kortex_driver::GetProductConfiguration::Response &res) override;
        virtual bool UpdateEndEffectorTypeConfiguration(kortex_driver::UpdateEndEffectorTypeConfiguration::Request  &req, kortex_driver::UpdateEndEffectorTypeConfiguration::Response &res) override;
        virtual bool RestoreFactoryProductConfiguration(kortex_driver::RestoreFactoryProductConfiguration::Request  &req, kortex_driver::RestoreFactoryProductConfiguration::Response &res) override;
        virtual bool GetTrajectoryErrorReport(kortex_driver::GetTrajectoryErrorReport::Request  &req, kortex_driver::GetTrajectoryErrorReport::Response &res) override;
        virtual bool GetAllJointsSpeedSoftLimitation(kortex_driver::GetAllJointsSpeedSoftLimitation::Request  &req, kortex_driver::GetAllJointsSpeedSoftLimitation::Response &res) override;
        virtual bool GetAllJointsTorqueSoftLimitation(kortex_driver::GetAllJointsTorqueSoftLimitation::Request  &req, kortex_driver::GetAllJointsTorqueSoftLimitation::Response &res) override;
        virtual bool GetTwistSoftLimitation(kortex_driver::GetTwistSoftLimitation::Request  &req, kortex_driver::GetTwistSoftLimitation::Response &res) override;
        virtual bool GetWrenchSoftLimitation(kortex_driver::GetWrenchSoftLimitation::Request  &req, kortex_driver::GetWrenchSoftLimitation::Response &res) override;
        virtual bool SetControllerConfigurationMode(kortex_driver::SetControllerConfigurationMode::Request  &req, kortex_driver::SetControllerConfigurationMode::Response &res) override;
        virtual bool GetControllerConfigurationMode(kortex_driver::GetControllerConfigurationMode::Request  &req, kortex_driver::GetControllerConfigurationMode::Response &res) override;
        virtual bool StartTeaching(kortex_driver::StartTeaching::Request  &req, kortex_driver::StartTeaching::Response &res) override;
        virtual bool StopTeaching(kortex_driver::StopTeaching::Request  &req, kortex_driver::StopTeaching::Response &res) override;
        virtual bool AddSequenceTasks(kortex_driver::AddSequenceTasks::Request  &req, kortex_driver::AddSequenceTasks::Response &res) override;
        virtual bool UpdateSequenceTask(kortex_driver::UpdateSequenceTask::Request  &req, kortex_driver::UpdateSequenceTask::Response &res) override;
        virtual bool SwapSequenceTasks(kortex_driver::SwapSequenceTasks::Request  &req, kortex_driver::SwapSequenceTasks::Response &res) override;
        virtual bool ReadSequenceTask(kortex_driver::ReadSequenceTask::Request  &req, kortex_driver::ReadSequenceTask::Response &res) override;
        virtual bool ReadAllSequenceTasks(kortex_driver::ReadAllSequenceTasks::Request  &req, kortex_driver::ReadAllSequenceTasks::Response &res) override;
        virtual bool DeleteSequenceTask(kortex_driver::DeleteSequenceTask::Request  &req, kortex_driver::DeleteSequenceTask::Response &res) override;
        virtual bool DeleteAllSequenceTasks(kortex_driver::DeleteAllSequenceTasks::Request  &req, kortex_driver::DeleteAllSequenceTasks::Response &res) override;
        virtual bool TakeSnapshot(kortex_driver::TakeSnapshot::Request  &req, kortex_driver::TakeSnapshot::Response &res) override;
        virtual bool GetFirmwareBundleVersions(kortex_driver::GetFirmwareBundleVersions::Request  &req, kortex_driver::GetFirmwareBundleVersions::Response &res) override;
        virtual bool ExecuteWaypointTrajectory(kortex_driver::ExecuteWaypointTrajectory::Request  &req, kortex_driver::ExecuteWaypointTrajectory::Response &res) override;
        virtual bool MoveSequenceTask(kortex_driver::MoveSequenceTask::Request  &req, kortex_driver::MoveSequenceTask::Response &res) override;
        virtual bool DuplicateMapping(kortex_driver::DuplicateMapping::Request  &req, kortex_driver::DuplicateMapping::Response &res) override;
        virtual bool DuplicateMap(kortex_driver::DuplicateMap::Request  &req, kortex_driver::DuplicateMap::Response &res) override;
        virtual bool SetControllerConfiguration(kortex_driver::SetControllerConfiguration::Request  &req, kortex_driver::SetControllerConfiguration::Response &res) override;
        virtual bool GetControllerConfiguration(kortex_driver::GetControllerConfiguration::Request  &req, kortex_driver::GetControllerConfiguration::Response &res) override;
        virtual bool GetAllControllerConfigurations(kortex_driver::GetAllControllerConfigurations::Request  &req, kortex_driver::GetAllControllerConfigurations::Response &res) override;
        virtual bool ComputeForwardKinematics(kortex_driver::ComputeForwardKinematics::Request  &req, kortex_driver::ComputeForwardKinematics::Response &res) override;
        virtual bool ComputeInverseKinematics(kortex_driver::ComputeInverseKinematics::Request  &req, kortex_driver::ComputeInverseKinematics::Response &res) override;
        virtual bool ValidateWaypointList(kortex_driver::ValidateWaypointList::Request  &req, kortex_driver::ValidateWaypointList::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::Base::BaseClient* m_base;
};
#endif
