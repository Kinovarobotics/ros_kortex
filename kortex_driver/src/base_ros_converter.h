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
 
#ifndef _KORTEX_BaseROS_CONVERTER_H_
#define _KORTEX_BaseROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <Base.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_driver/FullUserProfile.h"
#include "kortex_driver/UserProfile.h"
#include "kortex_driver/UserProfileList.h"
#include "kortex_driver/UserList.h"
#include "kortex_driver/PasswordChange.h"
#include "kortex_driver/SequenceHandle.h"
#include "kortex_driver/AdvancedSequenceHandle.h"
#include "kortex_driver/SequenceTaskHandle.h"
#include "kortex_driver/SequenceTask.h"
#include "kortex_driver/Sequence.h"
#include "kortex_driver/SequenceList.h"
#include "kortex_driver/AppendActionInformation.h"
#include "kortex_driver/ActionHandle.h"
#include "kortex_driver/RequestedActionType.h"
#include "kortex_driver/Action.h"
#include "kortex_driver/SwitchControlMapping.h"
#include "kortex_driver/ChangeTwist.h"
#include "kortex_driver/ChangeJointSpeeds.h"
#include "kortex_driver/EmergencyStop.h"
#include "kortex_driver/Faults.h"
#include "kortex_driver/Delay.h"
#include "kortex_driver/Stop.h"
#include "kortex_driver/ActionList.h"
#include "kortex_driver/Timeout.h"
#include "kortex_driver/Ssid.h"
#include "kortex_driver/CommunicationInterfaceConfiguration.h"
#include "kortex_driver/NetworkHandle.h"
#include "kortex_driver/IPv4Configuration.h"
#include "kortex_driver/IPv4Information.h"
#include "kortex_driver/FullIPv4Configuration.h"
#include "kortex_driver/WifiInformation.h"
#include "kortex_driver/WifiInformationList.h"
#include "kortex_driver/WifiConfiguration.h"
#include "kortex_driver/WifiConfigurationList.h"
#include "kortex_driver/ProtectionZoneHandle.h"
#include "kortex_driver/RotationMatrixRow.h"
#include "kortex_driver/RotationMatrix.h"
#include "kortex_driver/Point.h"
#include "kortex_driver/ZoneShape.h"
#include "kortex_driver/ProtectionZone.h"
#include "kortex_driver/ProtectionZoneList.h"
#include "kortex_driver/LimitationTypeIdentifier.h"
#include "kortex_driver/CartesianLimitation.h"
#include "kortex_driver/CartesianLimitationList.h"
#include "kortex_driver/JointLimitationValue.h"
#include "kortex_driver/JointLimitationValueList.h"
#include "kortex_driver/JointLimitation.h"
#include "kortex_driver/JointLimitationTypeIdentifier.h"
#include "kortex_driver/Query.h"
#include "kortex_driver/ConfigurationChangeNotification.h"
#include "kortex_driver/MappingInfoNotification.h"
#include "kortex_driver/ControlModeInformation.h"
#include "kortex_driver/ControlModeNotification.h"
#include "kortex_driver/ServoingModeInformation.h"
#include "kortex_driver/OperatingModeInformation.h"
#include "kortex_driver/OperatingModeNotification.h"
#include "kortex_driver/ServoingModeNotification.h"
#include "kortex_driver/SequenceInfoNotification.h"
#include "kortex_driver/SequenceInformation.h"
#include "kortex_driver/ProtectionZoneNotification.h"
#include "kortex_driver/ProtectionZoneInformation.h"
#include "kortex_driver/UserNotification.h"
#include "kortex_driver/ControllerHandle.h"
#include "kortex_driver/ControllerElementHandle.h"
#include "kortex_driver/ControllerNotification.h"
#include "kortex_driver/ControllerList.h"
#include "kortex_driver/ControllerState.h"
#include "kortex_driver/ControllerElementState.h"
#include "kortex_driver/ActionNotification.h"
#include "kortex_driver/ActionExecutionState.h"
#include "kortex_driver/RobotEventNotification.h"
#include "kortex_driver/FactoryNotification.h"
#include "kortex_driver/NetworkNotification.h"
#include "kortex_driver/ConfigurationChangeNotificationList.h"
#include "kortex_driver/MappingInfoNotificationList.h"
#include "kortex_driver/ControlModeNotificationList.h"
#include "kortex_driver/OperatingModeNotificationList.h"
#include "kortex_driver/ServoingModeNotificationList.h"
#include "kortex_driver/SequenceInfoNotificationList.h"
#include "kortex_driver/ProtectionZoneNotificationList.h"
#include "kortex_driver/UserNotificationList.h"
#include "kortex_driver/SafetyNotificationList.h"
#include "kortex_driver/ControllerNotificationList.h"
#include "kortex_driver/ActionNotificationList.h"
#include "kortex_driver/RobotEventNotificationList.h"
#include "kortex_driver/NetworkNotificationList.h"
#include "kortex_driver/MappingHandle.h"
#include "kortex_driver/SafetyEvent.h"
#include "kortex_driver/ControllerEvent.h"
#include "kortex_driver/GpioEvent.h"
#include "kortex_driver/MapEvent.h"
#include "kortex_driver/MapElement.h"
#include "kortex_driver/ActivateMapHandle.h"
#include "kortex_driver/Map.h"
#include "kortex_driver/MapHandle.h"
#include "kortex_driver/MapList.h"
#include "kortex_driver/MapGroupHandle.h"
#include "kortex_driver/MapGroup.h"
#include "kortex_driver/MapGroupList.h"
#include "kortex_driver/Mapping.h"
#include "kortex_driver/MappingList.h"
#include "kortex_driver/TransformationMatrix.h"
#include "kortex_driver/TransformationRow.h"
#include "kortex_driver/Pose.h"
#include "kortex_driver/Position.h"
#include "kortex_driver/Orientation.h"
#include "kortex_driver/CartesianSpeed.h"
#include "kortex_driver/CartesianTrajectoryConstraint.h"
#include "kortex_driver/JointTrajectoryConstraint.h"
#include "kortex_driver/Twist.h"
#include "kortex_driver/Admittance.h"
#include "kortex_driver/CartesianReferenceFrameRequest.h"
#include "kortex_driver/ConstrainedPose.h"
#include "kortex_driver/ConstrainedPosition.h"
#include "kortex_driver/ConstrainedOrientation.h"
#include "kortex_driver/TwistCommand.h"
#include "kortex_driver/ConstrainedJointAngles.h"
#include "kortex_driver/ConstrainedJointAngle.h"
#include "kortex_driver/JointAngles.h"
#include "kortex_driver/JointAngle.h"
#include "kortex_driver/JointSpeeds.h"
#include "kortex_driver/JointSpeed.h"
#include "kortex_driver/GripperCommand.h"
#include "kortex_driver/GripperRequest.h"
#include "kortex_driver/Gripper.h"
#include "kortex_driver/Finger.h"
#include "kortex_driver/SystemTime.h"
#include "kortex_driver/ActuatorInformation.h"
#include "kortex_driver/ArmStateInformation.h"
#include "kortex_driver/ArmStateNotification.h"
#include "kortex_driver/CountryCode.h"


using namespace Kinova::Api::Base;

int ToRosData(FullUserProfile input, kortex_driver::FullUserProfile &output);
int ToRosData(UserProfile input, kortex_driver::UserProfile &output);
int ToRosData(UserProfileList input, kortex_driver::UserProfileList &output);
int ToRosData(UserList input, kortex_driver::UserList &output);
int ToRosData(PasswordChange input, kortex_driver::PasswordChange &output);
int ToRosData(SequenceHandle input, kortex_driver::SequenceHandle &output);
int ToRosData(AdvancedSequenceHandle input, kortex_driver::AdvancedSequenceHandle &output);
int ToRosData(SequenceTaskHandle input, kortex_driver::SequenceTaskHandle &output);
int ToRosData(SequenceTask input, kortex_driver::SequenceTask &output);
int ToRosData(Sequence input, kortex_driver::Sequence &output);
int ToRosData(SequenceList input, kortex_driver::SequenceList &output);
int ToRosData(AppendActionInformation input, kortex_driver::AppendActionInformation &output);
int ToRosData(ActionHandle input, kortex_driver::ActionHandle &output);
int ToRosData(RequestedActionType input, kortex_driver::RequestedActionType &output);
int ToRosData(Action input, kortex_driver::Action &output);
int ToRosData(SwitchControlMapping input, kortex_driver::SwitchControlMapping &output);
int ToRosData(ChangeTwist input, kortex_driver::ChangeTwist &output);
int ToRosData(ChangeJointSpeeds input, kortex_driver::ChangeJointSpeeds &output);
int ToRosData(EmergencyStop input, kortex_driver::EmergencyStop &output);
int ToRosData(Faults input, kortex_driver::Faults &output);
int ToRosData(Delay input, kortex_driver::Delay &output);
int ToRosData(Stop input, kortex_driver::Stop &output);
int ToRosData(ActionList input, kortex_driver::ActionList &output);
int ToRosData(Timeout input, kortex_driver::Timeout &output);
int ToRosData(Ssid input, kortex_driver::Ssid &output);
int ToRosData(CommunicationInterfaceConfiguration input, kortex_driver::CommunicationInterfaceConfiguration &output);
int ToRosData(NetworkHandle input, kortex_driver::NetworkHandle &output);
int ToRosData(IPv4Configuration input, kortex_driver::IPv4Configuration &output);
int ToRosData(IPv4Information input, kortex_driver::IPv4Information &output);
int ToRosData(FullIPv4Configuration input, kortex_driver::FullIPv4Configuration &output);
int ToRosData(WifiInformation input, kortex_driver::WifiInformation &output);
int ToRosData(WifiInformationList input, kortex_driver::WifiInformationList &output);
int ToRosData(WifiConfiguration input, kortex_driver::WifiConfiguration &output);
int ToRosData(WifiConfigurationList input, kortex_driver::WifiConfigurationList &output);
int ToRosData(ProtectionZoneHandle input, kortex_driver::ProtectionZoneHandle &output);
int ToRosData(RotationMatrixRow input, kortex_driver::RotationMatrixRow &output);
int ToRosData(RotationMatrix input, kortex_driver::RotationMatrix &output);
int ToRosData(Point input, kortex_driver::Point &output);
int ToRosData(ZoneShape input, kortex_driver::ZoneShape &output);
int ToRosData(ProtectionZone input, kortex_driver::ProtectionZone &output);
int ToRosData(ProtectionZoneList input, kortex_driver::ProtectionZoneList &output);
int ToRosData(LimitationTypeIdentifier input, kortex_driver::LimitationTypeIdentifier &output);
int ToRosData(CartesianLimitation input, kortex_driver::CartesianLimitation &output);
int ToRosData(CartesianLimitationList input, kortex_driver::CartesianLimitationList &output);
int ToRosData(JointLimitationValue input, kortex_driver::JointLimitationValue &output);
int ToRosData(JointLimitationValueList input, kortex_driver::JointLimitationValueList &output);
int ToRosData(JointLimitation input, kortex_driver::JointLimitation &output);
int ToRosData(JointLimitationTypeIdentifier input, kortex_driver::JointLimitationTypeIdentifier &output);
int ToRosData(Query input, kortex_driver::Query &output);
int ToRosData(ConfigurationChangeNotification input, kortex_driver::ConfigurationChangeNotification &output);
int ToRosData(MappingInfoNotification input, kortex_driver::MappingInfoNotification &output);
int ToRosData(ControlModeInformation input, kortex_driver::ControlModeInformation &output);
int ToRosData(ControlModeNotification input, kortex_driver::ControlModeNotification &output);
int ToRosData(ServoingModeInformation input, kortex_driver::ServoingModeInformation &output);
int ToRosData(OperatingModeInformation input, kortex_driver::OperatingModeInformation &output);
int ToRosData(OperatingModeNotification input, kortex_driver::OperatingModeNotification &output);
int ToRosData(ServoingModeNotification input, kortex_driver::ServoingModeNotification &output);
int ToRosData(SequenceInfoNotification input, kortex_driver::SequenceInfoNotification &output);
int ToRosData(SequenceInformation input, kortex_driver::SequenceInformation &output);
int ToRosData(ProtectionZoneNotification input, kortex_driver::ProtectionZoneNotification &output);
int ToRosData(ProtectionZoneInformation input, kortex_driver::ProtectionZoneInformation &output);
int ToRosData(UserNotification input, kortex_driver::UserNotification &output);
int ToRosData(ControllerHandle input, kortex_driver::ControllerHandle &output);
int ToRosData(ControllerElementHandle input, kortex_driver::ControllerElementHandle &output);
int ToRosData(ControllerNotification input, kortex_driver::ControllerNotification &output);
int ToRosData(ControllerList input, kortex_driver::ControllerList &output);
int ToRosData(ControllerState input, kortex_driver::ControllerState &output);
int ToRosData(ControllerElementState input, kortex_driver::ControllerElementState &output);
int ToRosData(ActionNotification input, kortex_driver::ActionNotification &output);
int ToRosData(ActionExecutionState input, kortex_driver::ActionExecutionState &output);
int ToRosData(RobotEventNotification input, kortex_driver::RobotEventNotification &output);
int ToRosData(FactoryNotification input, kortex_driver::FactoryNotification &output);
int ToRosData(NetworkNotification input, kortex_driver::NetworkNotification &output);
int ToRosData(ConfigurationChangeNotificationList input, kortex_driver::ConfigurationChangeNotificationList &output);
int ToRosData(MappingInfoNotificationList input, kortex_driver::MappingInfoNotificationList &output);
int ToRosData(ControlModeNotificationList input, kortex_driver::ControlModeNotificationList &output);
int ToRosData(OperatingModeNotificationList input, kortex_driver::OperatingModeNotificationList &output);
int ToRosData(ServoingModeNotificationList input, kortex_driver::ServoingModeNotificationList &output);
int ToRosData(SequenceInfoNotificationList input, kortex_driver::SequenceInfoNotificationList &output);
int ToRosData(ProtectionZoneNotificationList input, kortex_driver::ProtectionZoneNotificationList &output);
int ToRosData(UserNotificationList input, kortex_driver::UserNotificationList &output);
int ToRosData(SafetyNotificationList input, kortex_driver::SafetyNotificationList &output);
int ToRosData(ControllerNotificationList input, kortex_driver::ControllerNotificationList &output);
int ToRosData(ActionNotificationList input, kortex_driver::ActionNotificationList &output);
int ToRosData(RobotEventNotificationList input, kortex_driver::RobotEventNotificationList &output);
int ToRosData(NetworkNotificationList input, kortex_driver::NetworkNotificationList &output);
int ToRosData(MappingHandle input, kortex_driver::MappingHandle &output);
int ToRosData(SafetyEvent input, kortex_driver::SafetyEvent &output);
int ToRosData(ControllerEvent input, kortex_driver::ControllerEvent &output);
int ToRosData(GpioEvent input, kortex_driver::GpioEvent &output);
int ToRosData(MapEvent input, kortex_driver::MapEvent &output);
int ToRosData(MapElement input, kortex_driver::MapElement &output);
int ToRosData(ActivateMapHandle input, kortex_driver::ActivateMapHandle &output);
int ToRosData(Map input, kortex_driver::Map &output);
int ToRosData(MapHandle input, kortex_driver::MapHandle &output);
int ToRosData(MapList input, kortex_driver::MapList &output);
int ToRosData(MapGroupHandle input, kortex_driver::MapGroupHandle &output);
int ToRosData(MapGroup input, kortex_driver::MapGroup &output);
int ToRosData(MapGroupList input, kortex_driver::MapGroupList &output);
int ToRosData(Mapping input, kortex_driver::Mapping &output);
int ToRosData(MappingList input, kortex_driver::MappingList &output);
int ToRosData(TransformationMatrix input, kortex_driver::TransformationMatrix &output);
int ToRosData(TransformationRow input, kortex_driver::TransformationRow &output);
int ToRosData(Pose input, kortex_driver::Pose &output);
int ToRosData(Position input, kortex_driver::Position &output);
int ToRosData(Orientation input, kortex_driver::Orientation &output);
int ToRosData(CartesianSpeed input, kortex_driver::CartesianSpeed &output);
int ToRosData(CartesianTrajectoryConstraint input, kortex_driver::CartesianTrajectoryConstraint &output);
int ToRosData(JointTrajectoryConstraint input, kortex_driver::JointTrajectoryConstraint &output);
int ToRosData(Twist input, kortex_driver::Twist &output);
int ToRosData(Admittance input, kortex_driver::Admittance &output);
int ToRosData(CartesianReferenceFrameRequest input, kortex_driver::CartesianReferenceFrameRequest &output);
int ToRosData(ConstrainedPose input, kortex_driver::ConstrainedPose &output);
int ToRosData(ConstrainedPosition input, kortex_driver::ConstrainedPosition &output);
int ToRosData(ConstrainedOrientation input, kortex_driver::ConstrainedOrientation &output);
int ToRosData(TwistCommand input, kortex_driver::TwistCommand &output);
int ToRosData(ConstrainedJointAngles input, kortex_driver::ConstrainedJointAngles &output);
int ToRosData(ConstrainedJointAngle input, kortex_driver::ConstrainedJointAngle &output);
int ToRosData(JointAngles input, kortex_driver::JointAngles &output);
int ToRosData(JointAngle input, kortex_driver::JointAngle &output);
int ToRosData(JointSpeeds input, kortex_driver::JointSpeeds &output);
int ToRosData(JointSpeed input, kortex_driver::JointSpeed &output);
int ToRosData(GripperCommand input, kortex_driver::GripperCommand &output);
int ToRosData(GripperRequest input, kortex_driver::GripperRequest &output);
int ToRosData(Gripper input, kortex_driver::Gripper &output);
int ToRosData(Finger input, kortex_driver::Finger &output);
int ToRosData(SystemTime input, kortex_driver::SystemTime &output);
int ToRosData(ActuatorInformation input, kortex_driver::ActuatorInformation &output);
int ToRosData(ArmStateInformation input, kortex_driver::ArmStateInformation &output);
int ToRosData(ArmStateNotification input, kortex_driver::ArmStateNotification &output);
int ToRosData(CountryCode input, kortex_driver::CountryCode &output);

#endif