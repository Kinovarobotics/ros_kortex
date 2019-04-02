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
 
#ifndef _KORTEX_BasePROTO_CONVERTER_H_
#define _KORTEX_BasePROTO_CONVERTER_H_

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

int ToProtoData(kortex_driver::FullUserProfile intput, FullUserProfile *output);
int ToProtoData(kortex_driver::UserProfile intput, UserProfile *output);
int ToProtoData(kortex_driver::UserProfileList intput, UserProfileList *output);
int ToProtoData(kortex_driver::UserList intput, UserList *output);
int ToProtoData(kortex_driver::PasswordChange intput, PasswordChange *output);
int ToProtoData(kortex_driver::SequenceHandle intput, SequenceHandle *output);
int ToProtoData(kortex_driver::AdvancedSequenceHandle intput, AdvancedSequenceHandle *output);
int ToProtoData(kortex_driver::SequenceTaskHandle intput, SequenceTaskHandle *output);
int ToProtoData(kortex_driver::SequenceTask intput, SequenceTask *output);
int ToProtoData(kortex_driver::Sequence intput, Sequence *output);
int ToProtoData(kortex_driver::SequenceList intput, SequenceList *output);
int ToProtoData(kortex_driver::AppendActionInformation intput, AppendActionInformation *output);
int ToProtoData(kortex_driver::ActionHandle intput, ActionHandle *output);
int ToProtoData(kortex_driver::RequestedActionType intput, RequestedActionType *output);
int ToProtoData(kortex_driver::Action intput, Action *output);
int ToProtoData(kortex_driver::SwitchControlMapping intput, SwitchControlMapping *output);
int ToProtoData(kortex_driver::ChangeTwist intput, ChangeTwist *output);
int ToProtoData(kortex_driver::ChangeJointSpeeds intput, ChangeJointSpeeds *output);
int ToProtoData(kortex_driver::EmergencyStop intput, EmergencyStop *output);
int ToProtoData(kortex_driver::Faults intput, Faults *output);
int ToProtoData(kortex_driver::Delay intput, Delay *output);
int ToProtoData(kortex_driver::Stop intput, Stop *output);
int ToProtoData(kortex_driver::ActionList intput, ActionList *output);
int ToProtoData(kortex_driver::Timeout intput, Timeout *output);
int ToProtoData(kortex_driver::Ssid intput, Ssid *output);
int ToProtoData(kortex_driver::CommunicationInterfaceConfiguration intput, CommunicationInterfaceConfiguration *output);
int ToProtoData(kortex_driver::NetworkHandle intput, NetworkHandle *output);
int ToProtoData(kortex_driver::IPv4Configuration intput, IPv4Configuration *output);
int ToProtoData(kortex_driver::IPv4Information intput, IPv4Information *output);
int ToProtoData(kortex_driver::FullIPv4Configuration intput, FullIPv4Configuration *output);
int ToProtoData(kortex_driver::WifiInformation intput, WifiInformation *output);
int ToProtoData(kortex_driver::WifiInformationList intput, WifiInformationList *output);
int ToProtoData(kortex_driver::WifiConfiguration intput, WifiConfiguration *output);
int ToProtoData(kortex_driver::WifiConfigurationList intput, WifiConfigurationList *output);
int ToProtoData(kortex_driver::ProtectionZoneHandle intput, ProtectionZoneHandle *output);
int ToProtoData(kortex_driver::RotationMatrixRow intput, RotationMatrixRow *output);
int ToProtoData(kortex_driver::RotationMatrix intput, RotationMatrix *output);
int ToProtoData(kortex_driver::Point intput, Point *output);
int ToProtoData(kortex_driver::ZoneShape intput, ZoneShape *output);
int ToProtoData(kortex_driver::ProtectionZone intput, ProtectionZone *output);
int ToProtoData(kortex_driver::ProtectionZoneList intput, ProtectionZoneList *output);
int ToProtoData(kortex_driver::LimitationTypeIdentifier intput, LimitationTypeIdentifier *output);
int ToProtoData(kortex_driver::CartesianLimitation intput, CartesianLimitation *output);
int ToProtoData(kortex_driver::CartesianLimitationList intput, CartesianLimitationList *output);
int ToProtoData(kortex_driver::JointLimitationValue intput, JointLimitationValue *output);
int ToProtoData(kortex_driver::JointLimitationValueList intput, JointLimitationValueList *output);
int ToProtoData(kortex_driver::JointLimitation intput, JointLimitation *output);
int ToProtoData(kortex_driver::JointLimitationTypeIdentifier intput, JointLimitationTypeIdentifier *output);
int ToProtoData(kortex_driver::Query intput, Query *output);
int ToProtoData(kortex_driver::ConfigurationChangeNotification intput, ConfigurationChangeNotification *output);
int ToProtoData(kortex_driver::MappingInfoNotification intput, MappingInfoNotification *output);
int ToProtoData(kortex_driver::ControlModeInformation intput, ControlModeInformation *output);
int ToProtoData(kortex_driver::ControlModeNotification intput, ControlModeNotification *output);
int ToProtoData(kortex_driver::ServoingModeInformation intput, ServoingModeInformation *output);
int ToProtoData(kortex_driver::OperatingModeInformation intput, OperatingModeInformation *output);
int ToProtoData(kortex_driver::OperatingModeNotification intput, OperatingModeNotification *output);
int ToProtoData(kortex_driver::ServoingModeNotification intput, ServoingModeNotification *output);
int ToProtoData(kortex_driver::SequenceInfoNotification intput, SequenceInfoNotification *output);
int ToProtoData(kortex_driver::SequenceInformation intput, SequenceInformation *output);
int ToProtoData(kortex_driver::ProtectionZoneNotification intput, ProtectionZoneNotification *output);
int ToProtoData(kortex_driver::ProtectionZoneInformation intput, ProtectionZoneInformation *output);
int ToProtoData(kortex_driver::UserNotification intput, UserNotification *output);
int ToProtoData(kortex_driver::ControllerHandle intput, ControllerHandle *output);
int ToProtoData(kortex_driver::ControllerElementHandle intput, ControllerElementHandle *output);
int ToProtoData(kortex_driver::ControllerNotification intput, ControllerNotification *output);
int ToProtoData(kortex_driver::ControllerList intput, ControllerList *output);
int ToProtoData(kortex_driver::ControllerState intput, ControllerState *output);
int ToProtoData(kortex_driver::ControllerElementState intput, ControllerElementState *output);
int ToProtoData(kortex_driver::ActionNotification intput, ActionNotification *output);
int ToProtoData(kortex_driver::ActionExecutionState intput, ActionExecutionState *output);
int ToProtoData(kortex_driver::RobotEventNotification intput, RobotEventNotification *output);
int ToProtoData(kortex_driver::FactoryNotification intput, FactoryNotification *output);
int ToProtoData(kortex_driver::NetworkNotification intput, NetworkNotification *output);
int ToProtoData(kortex_driver::ConfigurationChangeNotificationList intput, ConfigurationChangeNotificationList *output);
int ToProtoData(kortex_driver::MappingInfoNotificationList intput, MappingInfoNotificationList *output);
int ToProtoData(kortex_driver::ControlModeNotificationList intput, ControlModeNotificationList *output);
int ToProtoData(kortex_driver::OperatingModeNotificationList intput, OperatingModeNotificationList *output);
int ToProtoData(kortex_driver::ServoingModeNotificationList intput, ServoingModeNotificationList *output);
int ToProtoData(kortex_driver::SequenceInfoNotificationList intput, SequenceInfoNotificationList *output);
int ToProtoData(kortex_driver::ProtectionZoneNotificationList intput, ProtectionZoneNotificationList *output);
int ToProtoData(kortex_driver::UserNotificationList intput, UserNotificationList *output);
int ToProtoData(kortex_driver::SafetyNotificationList intput, SafetyNotificationList *output);
int ToProtoData(kortex_driver::ControllerNotificationList intput, ControllerNotificationList *output);
int ToProtoData(kortex_driver::ActionNotificationList intput, ActionNotificationList *output);
int ToProtoData(kortex_driver::RobotEventNotificationList intput, RobotEventNotificationList *output);
int ToProtoData(kortex_driver::NetworkNotificationList intput, NetworkNotificationList *output);
int ToProtoData(kortex_driver::MappingHandle intput, MappingHandle *output);
int ToProtoData(kortex_driver::SafetyEvent intput, SafetyEvent *output);
int ToProtoData(kortex_driver::ControllerEvent intput, ControllerEvent *output);
int ToProtoData(kortex_driver::GpioEvent intput, GpioEvent *output);
int ToProtoData(kortex_driver::MapEvent intput, MapEvent *output);
int ToProtoData(kortex_driver::MapElement intput, MapElement *output);
int ToProtoData(kortex_driver::ActivateMapHandle intput, ActivateMapHandle *output);
int ToProtoData(kortex_driver::Map intput, Map *output);
int ToProtoData(kortex_driver::MapHandle intput, MapHandle *output);
int ToProtoData(kortex_driver::MapList intput, MapList *output);
int ToProtoData(kortex_driver::MapGroupHandle intput, MapGroupHandle *output);
int ToProtoData(kortex_driver::MapGroup intput, MapGroup *output);
int ToProtoData(kortex_driver::MapGroupList intput, MapGroupList *output);
int ToProtoData(kortex_driver::Mapping intput, Mapping *output);
int ToProtoData(kortex_driver::MappingList intput, MappingList *output);
int ToProtoData(kortex_driver::TransformationMatrix intput, TransformationMatrix *output);
int ToProtoData(kortex_driver::TransformationRow intput, TransformationRow *output);
int ToProtoData(kortex_driver::Pose intput, Pose *output);
int ToProtoData(kortex_driver::Position intput, Position *output);
int ToProtoData(kortex_driver::Orientation intput, Orientation *output);
int ToProtoData(kortex_driver::CartesianSpeed intput, CartesianSpeed *output);
int ToProtoData(kortex_driver::CartesianTrajectoryConstraint intput, CartesianTrajectoryConstraint *output);
int ToProtoData(kortex_driver::JointTrajectoryConstraint intput, JointTrajectoryConstraint *output);
int ToProtoData(kortex_driver::Twist intput, Twist *output);
int ToProtoData(kortex_driver::Admittance intput, Admittance *output);
int ToProtoData(kortex_driver::CartesianReferenceFrameRequest intput, CartesianReferenceFrameRequest *output);
int ToProtoData(kortex_driver::ConstrainedPose intput, ConstrainedPose *output);
int ToProtoData(kortex_driver::ConstrainedPosition intput, ConstrainedPosition *output);
int ToProtoData(kortex_driver::ConstrainedOrientation intput, ConstrainedOrientation *output);
int ToProtoData(kortex_driver::TwistCommand intput, TwistCommand *output);
int ToProtoData(kortex_driver::ConstrainedJointAngles intput, ConstrainedJointAngles *output);
int ToProtoData(kortex_driver::ConstrainedJointAngle intput, ConstrainedJointAngle *output);
int ToProtoData(kortex_driver::JointAngles intput, JointAngles *output);
int ToProtoData(kortex_driver::JointAngle intput, JointAngle *output);
int ToProtoData(kortex_driver::JointSpeeds intput, JointSpeeds *output);
int ToProtoData(kortex_driver::JointSpeed intput, JointSpeed *output);
int ToProtoData(kortex_driver::GripperCommand intput, GripperCommand *output);
int ToProtoData(kortex_driver::GripperRequest intput, GripperRequest *output);
int ToProtoData(kortex_driver::Gripper intput, Gripper *output);
int ToProtoData(kortex_driver::Finger intput, Finger *output);
int ToProtoData(kortex_driver::SystemTime intput, SystemTime *output);
int ToProtoData(kortex_driver::ActuatorInformation intput, ActuatorInformation *output);
int ToProtoData(kortex_driver::ArmStateInformation intput, ArmStateInformation *output);
int ToProtoData(kortex_driver::ArmStateNotification intput, ArmStateNotification *output);
int ToProtoData(kortex_driver::CountryCode intput, CountryCode *output);

#endif