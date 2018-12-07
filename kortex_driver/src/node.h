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
 
#ifndef _KORTEX_SERVICES_H_
#define _KORTEX_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <Common.pb.h>
#include <BaseCyclic.pb.h>
#include <Errors.pb.h>
#include <Base.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>
#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include "kortex_driver/Refresh.h"
#include "kortex_driver/RefreshCommand.h"
#include "kortex_driver/RefreshFeedback.h"
#include "kortex_driver/RefreshCustomData.h"
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
#include "kortex_driver/Unsubscribe.h"
#include "kortex_driver/ConfigurationChangeTopic.h"
#include "kortex_driver/MappingInfoTopic.h"
#include "kortex_driver/ControlModeTopic.h"
#include "kortex_driver/OperatingModeTopic.h"
#include "kortex_driver/SequenceInfoTopic.h"
#include "kortex_driver/ProtectionZoneTopic.h"
#include "kortex_driver/UserTopic.h"
#include "kortex_driver/ControllerTopic.h"
#include "kortex_driver/ActionTopic.h"
#include "kortex_driver/RobotEventTopic.h"
#include "kortex_driver/GetFwdKinematics.h"
#include "kortex_driver/PlayCartesianTrajectory.h"
#include "kortex_driver/PlayCartesianTrajectoryPosition.h"
#include "kortex_driver/PlayCartesianTrajectoryOrientation.h"
#include "kortex_driver/Pause.h"
#include "kortex_driver/Resume.h"
#include "kortex_driver/GetMeasuredCartesianPose.h"
#include "kortex_driver/GetCommandedCartesianPose.h"
#include "kortex_driver/GetTargetedCartesianPose.h"
#include "kortex_driver/SendTwistCommand.h"
#include "kortex_driver/GetMeasuredTwist.h"
#include "kortex_driver/GetCommandedTwist.h"
#include "kortex_driver/PlayJointTrajectory.h"
#include "kortex_driver/PlaySelectedJointTrajectory.h"
#include "kortex_driver/GetMeasuredJointAngles.h"
#include "kortex_driver/GetCommandedJointAngles.h"
#include "kortex_driver/SendJointSpeedsCommmand.h"
#include "kortex_driver/SendSelectedJointSpeedCommand.h"
#include "kortex_driver/GetMeasuredJointSpeeds.h"
#include "kortex_driver/GetCommandedJointSpeeds.h"
#include "kortex_driver/SendGripperCommand.h"
#include "kortex_driver/GetMeasuredGripperMovement.h"
#include "kortex_driver/GetCommandedGripperMovement.h"
#include "kortex_driver/SetAdmittance.h"
#include "kortex_driver/SetTwistWrenchReferenceFrame.h"
#include "kortex_driver/SetOperatingMode.h"
#include "kortex_driver/ApplyEmergencyStop.h"
#include "kortex_driver/ClearFaults.h"
#include "kortex_driver/GetActiveMap.h"
#include "kortex_driver/GetControlMode.h"
#include "kortex_driver/GetOperatingMode.h"
#include "kortex_driver/SetServoingMode.h"
#include "kortex_driver/GetServoingMode.h"
#include "kortex_driver/ServoingModeTopic.h"
#include "kortex_driver/GetSequenceState.h"
#include "kortex_driver/GetProtectionZoneState.h"
#include "kortex_driver/GetActionExecutionState.h"
#include "kortex_driver/RestoreFactorySettings.h"
#include "kortex_driver/RestoreNetworkFactorySettings.h"
#include "kortex_driver/Reboot.h"
#include "kortex_driver/FactoryTopic.h"
#include "kortex_driver/GetAllConnectedControllers.h"
#include "kortex_driver/GetControllerState.h"
#include "kortex_driver/GetActuatorCount.h"
#include "kortex_driver/StartWifiScan.h"
#include "kortex_driver/GetConfiguredWifi.h"
#include "kortex_driver/NetworkTopic.h"
#include "kortex_driver/GetArmState.h"
#include "kortex_driver/ArmStateTopic.h"
#include "kortex_driver/GetIPv4Information.h"
#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"

#include "kortex_driver/ApiOptions.h"

using namespace std;
using namespace Kinova::Api;
using namespace Kinova::Api::Common;
using namespace Kinova::Api::BaseCyclic;
using namespace Kinova::Api;
using namespace Kinova::Api::Base;

class BaseServices
{
    public:
        BaseServices(char* ip, ros::NodeHandle& n);
        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);


        bool Refresh(kortex_driver::Refresh::Request  &req, kortex_driver::Refresh::Response &res);
        bool RefreshCommand(kortex_driver::RefreshCommand::Request  &req, kortex_driver::RefreshCommand::Response &res);
        bool RefreshFeedback(kortex_driver::RefreshFeedback::Request  &req, kortex_driver::RefreshFeedback::Response &res);
        bool RefreshCustomData(kortex_driver::RefreshCustomData::Request  &req, kortex_driver::RefreshCustomData::Response &res);


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
        bool Unsubscribe(kortex_driver::Unsubscribe::Request  &req, kortex_driver::Unsubscribe::Response &res);
        bool OnNotificationConfigurationChangeTopic(kortex_driver::ConfigurationChangeTopic::Request  &req, kortex_driver::ConfigurationChangeTopic::Response &res);
        void cb_ConfigurationChangeTopic(ConfigurationChangeNotification notif);
        bool OnNotificationMappingInfoTopic(kortex_driver::MappingInfoTopic::Request  &req, kortex_driver::MappingInfoTopic::Response &res);
        void cb_MappingInfoTopic(MappingInfoNotification notif);
        bool OnNotificationControlModeTopic(kortex_driver::ControlModeTopic::Request  &req, kortex_driver::ControlModeTopic::Response &res);
        void cb_ControlModeTopic(ControlModeNotification notif);
        bool OnNotificationOperatingModeTopic(kortex_driver::OperatingModeTopic::Request  &req, kortex_driver::OperatingModeTopic::Response &res);
        void cb_OperatingModeTopic(OperatingModeNotification notif);
        bool OnNotificationSequenceInfoTopic(kortex_driver::SequenceInfoTopic::Request  &req, kortex_driver::SequenceInfoTopic::Response &res);
        void cb_SequenceInfoTopic(SequenceInfoNotification notif);
        bool OnNotificationProtectionZoneTopic(kortex_driver::ProtectionZoneTopic::Request  &req, kortex_driver::ProtectionZoneTopic::Response &res);
        void cb_ProtectionZoneTopic(ProtectionZoneNotification notif);
        bool OnNotificationUserTopic(kortex_driver::UserTopic::Request  &req, kortex_driver::UserTopic::Response &res);
        void cb_UserTopic(UserNotification notif);
        bool OnNotificationControllerTopic(kortex_driver::ControllerTopic::Request  &req, kortex_driver::ControllerTopic::Response &res);
        void cb_ControllerTopic(ControllerNotification notif);
        bool OnNotificationActionTopic(kortex_driver::ActionTopic::Request  &req, kortex_driver::ActionTopic::Response &res);
        void cb_ActionTopic(ActionNotification notif);
        bool OnNotificationRobotEventTopic(kortex_driver::RobotEventTopic::Request  &req, kortex_driver::RobotEventTopic::Response &res);
        void cb_RobotEventTopic(RobotEventNotification notif);
        bool GetFwdKinematics(kortex_driver::GetFwdKinematics::Request  &req, kortex_driver::GetFwdKinematics::Response &res);
        bool PlayCartesianTrajectory(kortex_driver::PlayCartesianTrajectory::Request  &req, kortex_driver::PlayCartesianTrajectory::Response &res);
        bool PlayCartesianTrajectoryPosition(kortex_driver::PlayCartesianTrajectoryPosition::Request  &req, kortex_driver::PlayCartesianTrajectoryPosition::Response &res);
        bool PlayCartesianTrajectoryOrientation(kortex_driver::PlayCartesianTrajectoryOrientation::Request  &req, kortex_driver::PlayCartesianTrajectoryOrientation::Response &res);
        bool Pause(kortex_driver::Pause::Request  &req, kortex_driver::Pause::Response &res);
        bool Resume(kortex_driver::Resume::Request  &req, kortex_driver::Resume::Response &res);
        bool GetMeasuredCartesianPose(kortex_driver::GetMeasuredCartesianPose::Request  &req, kortex_driver::GetMeasuredCartesianPose::Response &res);
        bool GetCommandedCartesianPose(kortex_driver::GetCommandedCartesianPose::Request  &req, kortex_driver::GetCommandedCartesianPose::Response &res);
        bool GetTargetedCartesianPose(kortex_driver::GetTargetedCartesianPose::Request  &req, kortex_driver::GetTargetedCartesianPose::Response &res);
        bool SendTwistCommand(kortex_driver::SendTwistCommand::Request  &req, kortex_driver::SendTwistCommand::Response &res);
        bool GetMeasuredTwist(kortex_driver::GetMeasuredTwist::Request  &req, kortex_driver::GetMeasuredTwist::Response &res);
        bool GetCommandedTwist(kortex_driver::GetCommandedTwist::Request  &req, kortex_driver::GetCommandedTwist::Response &res);
        bool PlayJointTrajectory(kortex_driver::PlayJointTrajectory::Request  &req, kortex_driver::PlayJointTrajectory::Response &res);
        bool PlaySelectedJointTrajectory(kortex_driver::PlaySelectedJointTrajectory::Request  &req, kortex_driver::PlaySelectedJointTrajectory::Response &res);
        bool GetMeasuredJointAngles(kortex_driver::GetMeasuredJointAngles::Request  &req, kortex_driver::GetMeasuredJointAngles::Response &res);
        bool GetCommandedJointAngles(kortex_driver::GetCommandedJointAngles::Request  &req, kortex_driver::GetCommandedJointAngles::Response &res);
        bool SendJointSpeedsCommmand(kortex_driver::SendJointSpeedsCommmand::Request  &req, kortex_driver::SendJointSpeedsCommmand::Response &res);
        bool SendSelectedJointSpeedCommand(kortex_driver::SendSelectedJointSpeedCommand::Request  &req, kortex_driver::SendSelectedJointSpeedCommand::Response &res);
        bool GetMeasuredJointSpeeds(kortex_driver::GetMeasuredJointSpeeds::Request  &req, kortex_driver::GetMeasuredJointSpeeds::Response &res);
        bool GetCommandedJointSpeeds(kortex_driver::GetCommandedJointSpeeds::Request  &req, kortex_driver::GetCommandedJointSpeeds::Response &res);
        bool SendGripperCommand(kortex_driver::SendGripperCommand::Request  &req, kortex_driver::SendGripperCommand::Response &res);
        bool GetMeasuredGripperMovement(kortex_driver::GetMeasuredGripperMovement::Request  &req, kortex_driver::GetMeasuredGripperMovement::Response &res);
        bool GetCommandedGripperMovement(kortex_driver::GetCommandedGripperMovement::Request  &req, kortex_driver::GetCommandedGripperMovement::Response &res);
        bool SetAdmittance(kortex_driver::SetAdmittance::Request  &req, kortex_driver::SetAdmittance::Response &res);
        bool SetTwistWrenchReferenceFrame(kortex_driver::SetTwistWrenchReferenceFrame::Request  &req, kortex_driver::SetTwistWrenchReferenceFrame::Response &res);
        bool SetOperatingMode(kortex_driver::SetOperatingMode::Request  &req, kortex_driver::SetOperatingMode::Response &res);
        bool ApplyEmergencyStop(kortex_driver::ApplyEmergencyStop::Request  &req, kortex_driver::ApplyEmergencyStop::Response &res);
        bool ClearFaults(kortex_driver::ClearFaults::Request  &req, kortex_driver::ClearFaults::Response &res);
        bool GetActiveMap(kortex_driver::GetActiveMap::Request  &req, kortex_driver::GetActiveMap::Response &res);
        bool GetControlMode(kortex_driver::GetControlMode::Request  &req, kortex_driver::GetControlMode::Response &res);
        bool GetOperatingMode(kortex_driver::GetOperatingMode::Request  &req, kortex_driver::GetOperatingMode::Response &res);
        bool SetServoingMode(kortex_driver::SetServoingMode::Request  &req, kortex_driver::SetServoingMode::Response &res);
        bool GetServoingMode(kortex_driver::GetServoingMode::Request  &req, kortex_driver::GetServoingMode::Response &res);
        bool OnNotificationServoingModeTopic(kortex_driver::ServoingModeTopic::Request  &req, kortex_driver::ServoingModeTopic::Response &res);
        void cb_ServoingModeTopic(ServoingModeNotification notif);
        bool GetSequenceState(kortex_driver::GetSequenceState::Request  &req, kortex_driver::GetSequenceState::Response &res);
        bool GetProtectionZoneState(kortex_driver::GetProtectionZoneState::Request  &req, kortex_driver::GetProtectionZoneState::Response &res);
        bool GetActionExecutionState(kortex_driver::GetActionExecutionState::Request  &req, kortex_driver::GetActionExecutionState::Response &res);
        bool RestoreFactorySettings(kortex_driver::RestoreFactorySettings::Request  &req, kortex_driver::RestoreFactorySettings::Response &res);
        bool RestoreNetworkFactorySettings(kortex_driver::RestoreNetworkFactorySettings::Request  &req, kortex_driver::RestoreNetworkFactorySettings::Response &res);
        bool Reboot(kortex_driver::Reboot::Request  &req, kortex_driver::Reboot::Response &res);
        bool OnNotificationFactoryTopic(kortex_driver::FactoryTopic::Request  &req, kortex_driver::FactoryTopic::Response &res);
        void cb_FactoryTopic(FactoryNotification notif);
        bool GetAllConnectedControllers(kortex_driver::GetAllConnectedControllers::Request  &req, kortex_driver::GetAllConnectedControllers::Response &res);
        bool GetControllerState(kortex_driver::GetControllerState::Request  &req, kortex_driver::GetControllerState::Response &res);
        bool GetActuatorCount(kortex_driver::GetActuatorCount::Request  &req, kortex_driver::GetActuatorCount::Response &res);
        bool StartWifiScan(kortex_driver::StartWifiScan::Request  &req, kortex_driver::StartWifiScan::Response &res);
        bool GetConfiguredWifi(kortex_driver::GetConfiguredWifi::Request  &req, kortex_driver::GetConfiguredWifi::Response &res);
        bool OnNotificationNetworkTopic(kortex_driver::NetworkTopic::Request  &req, kortex_driver::NetworkTopic::Response &res);
        void cb_NetworkTopic(NetworkNotification notif);
        bool GetArmState(kortex_driver::GetArmState::Request  &req, kortex_driver::GetArmState::Response &res);
        bool OnNotificationArmStateTopic(kortex_driver::ArmStateTopic::Request  &req, kortex_driver::ArmStateTopic::Response &res);
        void cb_ArmStateTopic(ArmStateNotification notif);
        bool GetIPv4Information(kortex_driver::GetIPv4Information::Request  &req, kortex_driver::GetIPv4Information::Response &res);


private:
    	TransportClientUdp* m_transport;
    	RouterClient*       m_router;
        
        BaseCyclicClient*   m_basecyclic;
        BaseClient*   m_base;
        uint32_t m_CurrentDeviceID;
        RouterClientSendOptions m_apiOptions;

        SessionManager* m_SessionManager;

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
};
#endif
