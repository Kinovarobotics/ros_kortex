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
 
#ifndef _KORTEX_BASE_PROTO_CONVERTER_H_
#define _KORTEX_BASE_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Base.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/GpioConfigurationList.h"
#include "kortex_driver/Base_GpioConfiguration.h"
#include "kortex_driver/GpioPinConfiguration.h"
#include "kortex_driver/FullUserProfile.h"
#include "kortex_driver/UserProfile.h"
#include "kortex_driver/UserProfileList.h"
#include "kortex_driver/UserList.h"
#include "kortex_driver/PasswordChange.h"
#include "kortex_driver/SequenceHandle.h"
#include "kortex_driver/AdvancedSequenceHandle.h"
#include "kortex_driver/SequenceTaskHandle.h"
#include "kortex_driver/SequenceTask.h"
#include "kortex_driver/SequenceTasks.h"
#include "kortex_driver/SequenceTasksConfiguration.h"
#include "kortex_driver/SequenceTaskConfiguration.h"
#include "kortex_driver/SequenceTasksRange.h"
#include "kortex_driver/SequenceTasksPair.h"
#include "kortex_driver/Sequence.h"
#include "kortex_driver/SequenceList.h"
#include "kortex_driver/AppendActionInformation.h"
#include "kortex_driver/ActionHandle.h"
#include "kortex_driver/RequestedActionType.h"
#include "kortex_driver/Action.h"
#include "kortex_driver/Snapshot.h"
#include "kortex_driver/SwitchControlMapping.h"
#include "kortex_driver/ChangeTwist.h"
#include "kortex_driver/ChangeJointSpeeds.h"
#include "kortex_driver/ChangeWrench.h"
#include "kortex_driver/EmergencyStop.h"
#include "kortex_driver/Faults.h"
#include "kortex_driver/Delay.h"
#include "kortex_driver/Base_Stop.h"
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
#include "kortex_driver/Base_RotationMatrixRow.h"
#include "kortex_driver/Base_RotationMatrix.h"
#include "kortex_driver/Point.h"
#include "kortex_driver/ZoneShape.h"
#include "kortex_driver/ProtectionZone.h"
#include "kortex_driver/ProtectionZoneList.h"
#include "kortex_driver/CartesianLimitation.h"
#include "kortex_driver/TwistLimitation.h"
#include "kortex_driver/WrenchLimitation.h"
#include "kortex_driver/CartesianLimitationList.h"
#include "kortex_driver/JointLimitation.h"
#include "kortex_driver/JointsLimitationsList.h"
#include "kortex_driver/Query.h"
#include "kortex_driver/ConfigurationChangeNotification.h"
#include "kortex_driver/MappingInfoNotification.h"
#include "kortex_driver/Base_ControlModeInformation.h"
#include "kortex_driver/Base_ControlModeNotification.h"
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
#include "kortex_driver/TrajectoryInfo.h"
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
#include "kortex_driver/Base_Position.h"
#include "kortex_driver/Orientation.h"
#include "kortex_driver/CartesianSpeed.h"
#include "kortex_driver/CartesianTrajectoryConstraint.h"
#include "kortex_driver/JointTrajectoryConstraint.h"
#include "kortex_driver/Wrench.h"
#include "kortex_driver/Twist.h"
#include "kortex_driver/Admittance.h"
#include "kortex_driver/ConstrainedPose.h"
#include "kortex_driver/ConstrainedPosition.h"
#include "kortex_driver/ConstrainedOrientation.h"
#include "kortex_driver/WrenchCommand.h"
#include "kortex_driver/TwistCommand.h"
#include "kortex_driver/ConstrainedJointAngles.h"
#include "kortex_driver/ConstrainedJointAngle.h"
#include "kortex_driver/JointAngles.h"
#include "kortex_driver/JointAngle.h"
#include "kortex_driver/Base_JointSpeeds.h"
#include "kortex_driver/JointSpeed.h"
#include "kortex_driver/JointTorques.h"
#include "kortex_driver/JointTorque.h"
#include "kortex_driver/GripperCommand.h"
#include "kortex_driver/GripperRequest.h"
#include "kortex_driver/Gripper.h"
#include "kortex_driver/Finger.h"
#include "kortex_driver/GpioCommand.h"
#include "kortex_driver/SystemTime.h"
#include "kortex_driver/ControllerConfigurationMode.h"
#include "kortex_driver/ControllerConfiguration.h"
#include "kortex_driver/ControllerConfigurationList.h"
#include "kortex_driver/ActuatorInformation.h"
#include "kortex_driver/ArmStateInformation.h"
#include "kortex_driver/ArmStateNotification.h"
#include "kortex_driver/Base_CapSenseConfig.h"
#include "kortex_driver/BridgeList.h"
#include "kortex_driver/BridgeResult.h"
#include "kortex_driver/BridgeIdentifier.h"
#include "kortex_driver/BridgeConfig.h"
#include "kortex_driver/BridgePortConfig.h"
#include "kortex_driver/PreComputedJointTrajectory.h"
#include "kortex_driver/PreComputedJointTrajectoryElement.h"
#include "kortex_driver/TrajectoryErrorElement.h"
#include "kortex_driver/TrajectoryErrorReport.h"
#include "kortex_driver/WaypointValidationReport.h"
#include "kortex_driver/Waypoint.h"
#include "kortex_driver/AngularWaypoint.h"
#include "kortex_driver/CartesianWaypoint.h"
#include "kortex_driver/WaypointList.h"
#include "kortex_driver/KinematicTrajectoryConstraints.h"
#include "kortex_driver/FirmwareBundleVersions.h"
#include "kortex_driver/FirmwareComponentVersion.h"
#include "kortex_driver/IKData.h"


int ToProtoData(kortex_driver::GpioConfigurationList input, Kinova::Api::Base::GpioConfigurationList *output);
int ToProtoData(kortex_driver::Base_GpioConfiguration input, Kinova::Api::Base::GpioConfiguration *output);
int ToProtoData(kortex_driver::GpioPinConfiguration input, Kinova::Api::Base::GpioPinConfiguration *output);
int ToProtoData(kortex_driver::FullUserProfile input, Kinova::Api::Base::FullUserProfile *output);
int ToProtoData(kortex_driver::UserProfile input, Kinova::Api::Base::UserProfile *output);
int ToProtoData(kortex_driver::UserProfileList input, Kinova::Api::Base::UserProfileList *output);
int ToProtoData(kortex_driver::UserList input, Kinova::Api::Base::UserList *output);
int ToProtoData(kortex_driver::PasswordChange input, Kinova::Api::Base::PasswordChange *output);
int ToProtoData(kortex_driver::SequenceHandle input, Kinova::Api::Base::SequenceHandle *output);
int ToProtoData(kortex_driver::AdvancedSequenceHandle input, Kinova::Api::Base::AdvancedSequenceHandle *output);
int ToProtoData(kortex_driver::SequenceTaskHandle input, Kinova::Api::Base::SequenceTaskHandle *output);
int ToProtoData(kortex_driver::SequenceTask input, Kinova::Api::Base::SequenceTask *output);
int ToProtoData(kortex_driver::SequenceTasks input, Kinova::Api::Base::SequenceTasks *output);
int ToProtoData(kortex_driver::SequenceTasksConfiguration input, Kinova::Api::Base::SequenceTasksConfiguration *output);
int ToProtoData(kortex_driver::SequenceTaskConfiguration input, Kinova::Api::Base::SequenceTaskConfiguration *output);
int ToProtoData(kortex_driver::SequenceTasksRange input, Kinova::Api::Base::SequenceTasksRange *output);
int ToProtoData(kortex_driver::SequenceTasksPair input, Kinova::Api::Base::SequenceTasksPair *output);
int ToProtoData(kortex_driver::Sequence input, Kinova::Api::Base::Sequence *output);
int ToProtoData(kortex_driver::SequenceList input, Kinova::Api::Base::SequenceList *output);
int ToProtoData(kortex_driver::AppendActionInformation input, Kinova::Api::Base::AppendActionInformation *output);
int ToProtoData(kortex_driver::ActionHandle input, Kinova::Api::Base::ActionHandle *output);
int ToProtoData(kortex_driver::RequestedActionType input, Kinova::Api::Base::RequestedActionType *output);
int ToProtoData(kortex_driver::Action input, Kinova::Api::Base::Action *output);
int ToProtoData(kortex_driver::Snapshot input, Kinova::Api::Base::Snapshot *output);
int ToProtoData(kortex_driver::SwitchControlMapping input, Kinova::Api::Base::SwitchControlMapping *output);
int ToProtoData(kortex_driver::ChangeTwist input, Kinova::Api::Base::ChangeTwist *output);
int ToProtoData(kortex_driver::ChangeJointSpeeds input, Kinova::Api::Base::ChangeJointSpeeds *output);
int ToProtoData(kortex_driver::ChangeWrench input, Kinova::Api::Base::ChangeWrench *output);
int ToProtoData(kortex_driver::EmergencyStop input, Kinova::Api::Base::EmergencyStop *output);
int ToProtoData(kortex_driver::Faults input, Kinova::Api::Base::Faults *output);
int ToProtoData(kortex_driver::Delay input, Kinova::Api::Base::Delay *output);
int ToProtoData(kortex_driver::Base_Stop input, Kinova::Api::Base::Stop *output);
int ToProtoData(kortex_driver::ActionList input, Kinova::Api::Base::ActionList *output);
int ToProtoData(kortex_driver::Timeout input, Kinova::Api::Base::Timeout *output);
int ToProtoData(kortex_driver::Ssid input, Kinova::Api::Base::Ssid *output);
int ToProtoData(kortex_driver::CommunicationInterfaceConfiguration input, Kinova::Api::Base::CommunicationInterfaceConfiguration *output);
int ToProtoData(kortex_driver::NetworkHandle input, Kinova::Api::Base::NetworkHandle *output);
int ToProtoData(kortex_driver::IPv4Configuration input, Kinova::Api::Base::IPv4Configuration *output);
int ToProtoData(kortex_driver::IPv4Information input, Kinova::Api::Base::IPv4Information *output);
int ToProtoData(kortex_driver::FullIPv4Configuration input, Kinova::Api::Base::FullIPv4Configuration *output);
int ToProtoData(kortex_driver::WifiInformation input, Kinova::Api::Base::WifiInformation *output);
int ToProtoData(kortex_driver::WifiInformationList input, Kinova::Api::Base::WifiInformationList *output);
int ToProtoData(kortex_driver::WifiConfiguration input, Kinova::Api::Base::WifiConfiguration *output);
int ToProtoData(kortex_driver::WifiConfigurationList input, Kinova::Api::Base::WifiConfigurationList *output);
int ToProtoData(kortex_driver::ProtectionZoneHandle input, Kinova::Api::Base::ProtectionZoneHandle *output);
int ToProtoData(kortex_driver::Base_RotationMatrixRow input, Kinova::Api::Base::RotationMatrixRow *output);
int ToProtoData(kortex_driver::Base_RotationMatrix input, Kinova::Api::Base::RotationMatrix *output);
int ToProtoData(kortex_driver::Point input, Kinova::Api::Base::Point *output);
int ToProtoData(kortex_driver::ZoneShape input, Kinova::Api::Base::ZoneShape *output);
int ToProtoData(kortex_driver::ProtectionZone input, Kinova::Api::Base::ProtectionZone *output);
int ToProtoData(kortex_driver::ProtectionZoneList input, Kinova::Api::Base::ProtectionZoneList *output);
int ToProtoData(kortex_driver::CartesianLimitation input, Kinova::Api::Base::CartesianLimitation *output);
int ToProtoData(kortex_driver::TwistLimitation input, Kinova::Api::Base::TwistLimitation *output);
int ToProtoData(kortex_driver::WrenchLimitation input, Kinova::Api::Base::WrenchLimitation *output);
int ToProtoData(kortex_driver::CartesianLimitationList input, Kinova::Api::Base::CartesianLimitationList *output);
int ToProtoData(kortex_driver::JointLimitation input, Kinova::Api::Base::JointLimitation *output);
int ToProtoData(kortex_driver::JointsLimitationsList input, Kinova::Api::Base::JointsLimitationsList *output);
int ToProtoData(kortex_driver::Query input, Kinova::Api::Base::Query *output);
int ToProtoData(kortex_driver::ConfigurationChangeNotification input, Kinova::Api::Base::ConfigurationChangeNotification *output);
int ToProtoData(kortex_driver::MappingInfoNotification input, Kinova::Api::Base::MappingInfoNotification *output);
int ToProtoData(kortex_driver::Base_ControlModeInformation input, Kinova::Api::Base::ControlModeInformation *output);
int ToProtoData(kortex_driver::Base_ControlModeNotification input, Kinova::Api::Base::ControlModeNotification *output);
int ToProtoData(kortex_driver::ServoingModeInformation input, Kinova::Api::Base::ServoingModeInformation *output);
int ToProtoData(kortex_driver::OperatingModeInformation input, Kinova::Api::Base::OperatingModeInformation *output);
int ToProtoData(kortex_driver::OperatingModeNotification input, Kinova::Api::Base::OperatingModeNotification *output);
int ToProtoData(kortex_driver::ServoingModeNotification input, Kinova::Api::Base::ServoingModeNotification *output);
int ToProtoData(kortex_driver::SequenceInfoNotification input, Kinova::Api::Base::SequenceInfoNotification *output);
int ToProtoData(kortex_driver::SequenceInformation input, Kinova::Api::Base::SequenceInformation *output);
int ToProtoData(kortex_driver::ProtectionZoneNotification input, Kinova::Api::Base::ProtectionZoneNotification *output);
int ToProtoData(kortex_driver::ProtectionZoneInformation input, Kinova::Api::Base::ProtectionZoneInformation *output);
int ToProtoData(kortex_driver::UserNotification input, Kinova::Api::Base::UserNotification *output);
int ToProtoData(kortex_driver::ControllerHandle input, Kinova::Api::Base::ControllerHandle *output);
int ToProtoData(kortex_driver::ControllerElementHandle input, Kinova::Api::Base::ControllerElementHandle *output);
int ToProtoData(kortex_driver::ControllerNotification input, Kinova::Api::Base::ControllerNotification *output);
int ToProtoData(kortex_driver::ControllerList input, Kinova::Api::Base::ControllerList *output);
int ToProtoData(kortex_driver::ControllerState input, Kinova::Api::Base::ControllerState *output);
int ToProtoData(kortex_driver::ControllerElementState input, Kinova::Api::Base::ControllerElementState *output);
int ToProtoData(kortex_driver::ActionNotification input, Kinova::Api::Base::ActionNotification *output);
int ToProtoData(kortex_driver::TrajectoryInfo input, Kinova::Api::Base::TrajectoryInfo *output);
int ToProtoData(kortex_driver::ActionExecutionState input, Kinova::Api::Base::ActionExecutionState *output);
int ToProtoData(kortex_driver::RobotEventNotification input, Kinova::Api::Base::RobotEventNotification *output);
int ToProtoData(kortex_driver::FactoryNotification input, Kinova::Api::Base::FactoryNotification *output);
int ToProtoData(kortex_driver::NetworkNotification input, Kinova::Api::Base::NetworkNotification *output);
int ToProtoData(kortex_driver::ConfigurationChangeNotificationList input, Kinova::Api::Base::ConfigurationChangeNotificationList *output);
int ToProtoData(kortex_driver::MappingInfoNotificationList input, Kinova::Api::Base::MappingInfoNotificationList *output);
int ToProtoData(kortex_driver::ControlModeNotificationList input, Kinova::Api::Base::ControlModeNotificationList *output);
int ToProtoData(kortex_driver::OperatingModeNotificationList input, Kinova::Api::Base::OperatingModeNotificationList *output);
int ToProtoData(kortex_driver::ServoingModeNotificationList input, Kinova::Api::Base::ServoingModeNotificationList *output);
int ToProtoData(kortex_driver::SequenceInfoNotificationList input, Kinova::Api::Base::SequenceInfoNotificationList *output);
int ToProtoData(kortex_driver::ProtectionZoneNotificationList input, Kinova::Api::Base::ProtectionZoneNotificationList *output);
int ToProtoData(kortex_driver::UserNotificationList input, Kinova::Api::Base::UserNotificationList *output);
int ToProtoData(kortex_driver::SafetyNotificationList input, Kinova::Api::Base::SafetyNotificationList *output);
int ToProtoData(kortex_driver::ControllerNotificationList input, Kinova::Api::Base::ControllerNotificationList *output);
int ToProtoData(kortex_driver::ActionNotificationList input, Kinova::Api::Base::ActionNotificationList *output);
int ToProtoData(kortex_driver::RobotEventNotificationList input, Kinova::Api::Base::RobotEventNotificationList *output);
int ToProtoData(kortex_driver::NetworkNotificationList input, Kinova::Api::Base::NetworkNotificationList *output);
int ToProtoData(kortex_driver::MappingHandle input, Kinova::Api::Base::MappingHandle *output);
int ToProtoData(kortex_driver::SafetyEvent input, Kinova::Api::Base::SafetyEvent *output);
int ToProtoData(kortex_driver::ControllerEvent input, Kinova::Api::Base::ControllerEvent *output);
int ToProtoData(kortex_driver::GpioEvent input, Kinova::Api::Base::GpioEvent *output);
int ToProtoData(kortex_driver::MapEvent input, Kinova::Api::Base::MapEvent *output);
int ToProtoData(kortex_driver::MapElement input, Kinova::Api::Base::MapElement *output);
int ToProtoData(kortex_driver::ActivateMapHandle input, Kinova::Api::Base::ActivateMapHandle *output);
int ToProtoData(kortex_driver::Map input, Kinova::Api::Base::Map *output);
int ToProtoData(kortex_driver::MapHandle input, Kinova::Api::Base::MapHandle *output);
int ToProtoData(kortex_driver::MapList input, Kinova::Api::Base::MapList *output);
int ToProtoData(kortex_driver::MapGroupHandle input, Kinova::Api::Base::MapGroupHandle *output);
int ToProtoData(kortex_driver::MapGroup input, Kinova::Api::Base::MapGroup *output);
int ToProtoData(kortex_driver::MapGroupList input, Kinova::Api::Base::MapGroupList *output);
int ToProtoData(kortex_driver::Mapping input, Kinova::Api::Base::Mapping *output);
int ToProtoData(kortex_driver::MappingList input, Kinova::Api::Base::MappingList *output);
int ToProtoData(kortex_driver::TransformationMatrix input, Kinova::Api::Base::TransformationMatrix *output);
int ToProtoData(kortex_driver::TransformationRow input, Kinova::Api::Base::TransformationRow *output);
int ToProtoData(kortex_driver::Pose input, Kinova::Api::Base::Pose *output);
int ToProtoData(kortex_driver::Base_Position input, Kinova::Api::Base::Position *output);
int ToProtoData(kortex_driver::Orientation input, Kinova::Api::Base::Orientation *output);
int ToProtoData(kortex_driver::CartesianSpeed input, Kinova::Api::Base::CartesianSpeed *output);
int ToProtoData(kortex_driver::CartesianTrajectoryConstraint input, Kinova::Api::Base::CartesianTrajectoryConstraint *output);
int ToProtoData(kortex_driver::JointTrajectoryConstraint input, Kinova::Api::Base::JointTrajectoryConstraint *output);
int ToProtoData(kortex_driver::Wrench input, Kinova::Api::Base::Wrench *output);
int ToProtoData(kortex_driver::Twist input, Kinova::Api::Base::Twist *output);
int ToProtoData(kortex_driver::Admittance input, Kinova::Api::Base::Admittance *output);
int ToProtoData(kortex_driver::ConstrainedPose input, Kinova::Api::Base::ConstrainedPose *output);
int ToProtoData(kortex_driver::ConstrainedPosition input, Kinova::Api::Base::ConstrainedPosition *output);
int ToProtoData(kortex_driver::ConstrainedOrientation input, Kinova::Api::Base::ConstrainedOrientation *output);
int ToProtoData(kortex_driver::WrenchCommand input, Kinova::Api::Base::WrenchCommand *output);
int ToProtoData(kortex_driver::TwistCommand input, Kinova::Api::Base::TwistCommand *output);
int ToProtoData(kortex_driver::ConstrainedJointAngles input, Kinova::Api::Base::ConstrainedJointAngles *output);
int ToProtoData(kortex_driver::ConstrainedJointAngle input, Kinova::Api::Base::ConstrainedJointAngle *output);
int ToProtoData(kortex_driver::JointAngles input, Kinova::Api::Base::JointAngles *output);
int ToProtoData(kortex_driver::JointAngle input, Kinova::Api::Base::JointAngle *output);
int ToProtoData(kortex_driver::Base_JointSpeeds input, Kinova::Api::Base::JointSpeeds *output);
int ToProtoData(kortex_driver::JointSpeed input, Kinova::Api::Base::JointSpeed *output);
int ToProtoData(kortex_driver::JointTorques input, Kinova::Api::Base::JointTorques *output);
int ToProtoData(kortex_driver::JointTorque input, Kinova::Api::Base::JointTorque *output);
int ToProtoData(kortex_driver::GripperCommand input, Kinova::Api::Base::GripperCommand *output);
int ToProtoData(kortex_driver::GripperRequest input, Kinova::Api::Base::GripperRequest *output);
int ToProtoData(kortex_driver::Gripper input, Kinova::Api::Base::Gripper *output);
int ToProtoData(kortex_driver::Finger input, Kinova::Api::Base::Finger *output);
int ToProtoData(kortex_driver::GpioCommand input, Kinova::Api::Base::GpioCommand *output);
int ToProtoData(kortex_driver::SystemTime input, Kinova::Api::Base::SystemTime *output);
int ToProtoData(kortex_driver::ControllerConfigurationMode input, Kinova::Api::Base::ControllerConfigurationMode *output);
int ToProtoData(kortex_driver::ControllerConfiguration input, Kinova::Api::Base::ControllerConfiguration *output);
int ToProtoData(kortex_driver::ControllerConfigurationList input, Kinova::Api::Base::ControllerConfigurationList *output);
int ToProtoData(kortex_driver::ActuatorInformation input, Kinova::Api::Base::ActuatorInformation *output);
int ToProtoData(kortex_driver::ArmStateInformation input, Kinova::Api::Base::ArmStateInformation *output);
int ToProtoData(kortex_driver::ArmStateNotification input, Kinova::Api::Base::ArmStateNotification *output);
int ToProtoData(kortex_driver::Base_CapSenseConfig input, Kinova::Api::Base::CapSenseConfig *output);
int ToProtoData(kortex_driver::BridgeList input, Kinova::Api::Base::BridgeList *output);
int ToProtoData(kortex_driver::BridgeResult input, Kinova::Api::Base::BridgeResult *output);
int ToProtoData(kortex_driver::BridgeIdentifier input, Kinova::Api::Base::BridgeIdentifier *output);
int ToProtoData(kortex_driver::BridgeConfig input, Kinova::Api::Base::BridgeConfig *output);
int ToProtoData(kortex_driver::BridgePortConfig input, Kinova::Api::Base::BridgePortConfig *output);
int ToProtoData(kortex_driver::PreComputedJointTrajectory input, Kinova::Api::Base::PreComputedJointTrajectory *output);
int ToProtoData(kortex_driver::PreComputedJointTrajectoryElement input, Kinova::Api::Base::PreComputedJointTrajectoryElement *output);
int ToProtoData(kortex_driver::TrajectoryErrorElement input, Kinova::Api::Base::TrajectoryErrorElement *output);
int ToProtoData(kortex_driver::TrajectoryErrorReport input, Kinova::Api::Base::TrajectoryErrorReport *output);
int ToProtoData(kortex_driver::WaypointValidationReport input, Kinova::Api::Base::WaypointValidationReport *output);
int ToProtoData(kortex_driver::Waypoint input, Kinova::Api::Base::Waypoint *output);
int ToProtoData(kortex_driver::AngularWaypoint input, Kinova::Api::Base::AngularWaypoint *output);
int ToProtoData(kortex_driver::CartesianWaypoint input, Kinova::Api::Base::CartesianWaypoint *output);
int ToProtoData(kortex_driver::WaypointList input, Kinova::Api::Base::WaypointList *output);
int ToProtoData(kortex_driver::KinematicTrajectoryConstraints input, Kinova::Api::Base::KinematicTrajectoryConstraints *output);
int ToProtoData(kortex_driver::FirmwareBundleVersions input, Kinova::Api::Base::FirmwareBundleVersions *output);
int ToProtoData(kortex_driver::FirmwareComponentVersion input, Kinova::Api::Base::FirmwareComponentVersion *output);
int ToProtoData(kortex_driver::IKData input, Kinova::Api::Base::IKData *output);

#endif