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
 
#ifndef _KORTEX_BASE_ROS_CONVERTER_H_
#define _KORTEX_BASE_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Base.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


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


int ToRosData(Kinova::Api::Base::GpioConfigurationList input, kortex_driver::GpioConfigurationList &output);
int ToRosData(Kinova::Api::Base::GpioConfiguration input, kortex_driver::Base_GpioConfiguration &output);
int ToRosData(Kinova::Api::Base::GpioPinConfiguration input, kortex_driver::GpioPinConfiguration &output);
int ToRosData(Kinova::Api::Base::FullUserProfile input, kortex_driver::FullUserProfile &output);
int ToRosData(Kinova::Api::Base::UserProfile input, kortex_driver::UserProfile &output);
int ToRosData(Kinova::Api::Base::UserProfileList input, kortex_driver::UserProfileList &output);
int ToRosData(Kinova::Api::Base::UserList input, kortex_driver::UserList &output);
int ToRosData(Kinova::Api::Base::PasswordChange input, kortex_driver::PasswordChange &output);
int ToRosData(Kinova::Api::Base::SequenceHandle input, kortex_driver::SequenceHandle &output);
int ToRosData(Kinova::Api::Base::AdvancedSequenceHandle input, kortex_driver::AdvancedSequenceHandle &output);
int ToRosData(Kinova::Api::Base::SequenceTaskHandle input, kortex_driver::SequenceTaskHandle &output);
int ToRosData(Kinova::Api::Base::SequenceTask input, kortex_driver::SequenceTask &output);
int ToRosData(Kinova::Api::Base::SequenceTasks input, kortex_driver::SequenceTasks &output);
int ToRosData(Kinova::Api::Base::SequenceTasksConfiguration input, kortex_driver::SequenceTasksConfiguration &output);
int ToRosData(Kinova::Api::Base::SequenceTaskConfiguration input, kortex_driver::SequenceTaskConfiguration &output);
int ToRosData(Kinova::Api::Base::SequenceTasksRange input, kortex_driver::SequenceTasksRange &output);
int ToRosData(Kinova::Api::Base::SequenceTasksPair input, kortex_driver::SequenceTasksPair &output);
int ToRosData(Kinova::Api::Base::Sequence input, kortex_driver::Sequence &output);
int ToRosData(Kinova::Api::Base::SequenceList input, kortex_driver::SequenceList &output);
int ToRosData(Kinova::Api::Base::AppendActionInformation input, kortex_driver::AppendActionInformation &output);
int ToRosData(Kinova::Api::Base::ActionHandle input, kortex_driver::ActionHandle &output);
int ToRosData(Kinova::Api::Base::RequestedActionType input, kortex_driver::RequestedActionType &output);
int ToRosData(Kinova::Api::Base::Action input, kortex_driver::Action &output);
int ToRosData(Kinova::Api::Base::Snapshot input, kortex_driver::Snapshot &output);
int ToRosData(Kinova::Api::Base::SwitchControlMapping input, kortex_driver::SwitchControlMapping &output);
int ToRosData(Kinova::Api::Base::ChangeTwist input, kortex_driver::ChangeTwist &output);
int ToRosData(Kinova::Api::Base::ChangeJointSpeeds input, kortex_driver::ChangeJointSpeeds &output);
int ToRosData(Kinova::Api::Base::ChangeWrench input, kortex_driver::ChangeWrench &output);
int ToRosData(Kinova::Api::Base::EmergencyStop input, kortex_driver::EmergencyStop &output);
int ToRosData(Kinova::Api::Base::Faults input, kortex_driver::Faults &output);
int ToRosData(Kinova::Api::Base::Delay input, kortex_driver::Delay &output);
int ToRosData(Kinova::Api::Base::Stop input, kortex_driver::Base_Stop &output);
int ToRosData(Kinova::Api::Base::ActionList input, kortex_driver::ActionList &output);
int ToRosData(Kinova::Api::Base::Timeout input, kortex_driver::Timeout &output);
int ToRosData(Kinova::Api::Base::Ssid input, kortex_driver::Ssid &output);
int ToRosData(Kinova::Api::Base::CommunicationInterfaceConfiguration input, kortex_driver::CommunicationInterfaceConfiguration &output);
int ToRosData(Kinova::Api::Base::NetworkHandle input, kortex_driver::NetworkHandle &output);
int ToRosData(Kinova::Api::Base::IPv4Configuration input, kortex_driver::IPv4Configuration &output);
int ToRosData(Kinova::Api::Base::IPv4Information input, kortex_driver::IPv4Information &output);
int ToRosData(Kinova::Api::Base::FullIPv4Configuration input, kortex_driver::FullIPv4Configuration &output);
int ToRosData(Kinova::Api::Base::WifiInformation input, kortex_driver::WifiInformation &output);
int ToRosData(Kinova::Api::Base::WifiInformationList input, kortex_driver::WifiInformationList &output);
int ToRosData(Kinova::Api::Base::WifiConfiguration input, kortex_driver::WifiConfiguration &output);
int ToRosData(Kinova::Api::Base::WifiConfigurationList input, kortex_driver::WifiConfigurationList &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneHandle input, kortex_driver::ProtectionZoneHandle &output);
int ToRosData(Kinova::Api::Base::RotationMatrixRow input, kortex_driver::Base_RotationMatrixRow &output);
int ToRosData(Kinova::Api::Base::RotationMatrix input, kortex_driver::Base_RotationMatrix &output);
int ToRosData(Kinova::Api::Base::Point input, kortex_driver::Point &output);
int ToRosData(Kinova::Api::Base::ZoneShape input, kortex_driver::ZoneShape &output);
int ToRosData(Kinova::Api::Base::ProtectionZone input, kortex_driver::ProtectionZone &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneList input, kortex_driver::ProtectionZoneList &output);
int ToRosData(Kinova::Api::Base::CartesianLimitation input, kortex_driver::CartesianLimitation &output);
int ToRosData(Kinova::Api::Base::TwistLimitation input, kortex_driver::TwistLimitation &output);
int ToRosData(Kinova::Api::Base::WrenchLimitation input, kortex_driver::WrenchLimitation &output);
int ToRosData(Kinova::Api::Base::CartesianLimitationList input, kortex_driver::CartesianLimitationList &output);
int ToRosData(Kinova::Api::Base::JointLimitation input, kortex_driver::JointLimitation &output);
int ToRosData(Kinova::Api::Base::JointsLimitationsList input, kortex_driver::JointsLimitationsList &output);
int ToRosData(Kinova::Api::Base::Query input, kortex_driver::Query &output);
int ToRosData(Kinova::Api::Base::ConfigurationChangeNotification input, kortex_driver::ConfigurationChangeNotification &output);
int ToRosData(Kinova::Api::Base::MappingInfoNotification input, kortex_driver::MappingInfoNotification &output);
int ToRosData(Kinova::Api::Base::ControlModeInformation input, kortex_driver::Base_ControlModeInformation &output);
int ToRosData(Kinova::Api::Base::ControlModeNotification input, kortex_driver::Base_ControlModeNotification &output);
int ToRosData(Kinova::Api::Base::ServoingModeInformation input, kortex_driver::ServoingModeInformation &output);
int ToRosData(Kinova::Api::Base::OperatingModeInformation input, kortex_driver::OperatingModeInformation &output);
int ToRosData(Kinova::Api::Base::OperatingModeNotification input, kortex_driver::OperatingModeNotification &output);
int ToRosData(Kinova::Api::Base::ServoingModeNotification input, kortex_driver::ServoingModeNotification &output);
int ToRosData(Kinova::Api::Base::SequenceInfoNotification input, kortex_driver::SequenceInfoNotification &output);
int ToRosData(Kinova::Api::Base::SequenceInformation input, kortex_driver::SequenceInformation &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneNotification input, kortex_driver::ProtectionZoneNotification &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneInformation input, kortex_driver::ProtectionZoneInformation &output);
int ToRosData(Kinova::Api::Base::UserNotification input, kortex_driver::UserNotification &output);
int ToRosData(Kinova::Api::Base::ControllerHandle input, kortex_driver::ControllerHandle &output);
int ToRosData(Kinova::Api::Base::ControllerElementHandle input, kortex_driver::ControllerElementHandle &output);
int ToRosData(Kinova::Api::Base::ControllerNotification input, kortex_driver::ControllerNotification &output);
int ToRosData(Kinova::Api::Base::ControllerList input, kortex_driver::ControllerList &output);
int ToRosData(Kinova::Api::Base::ControllerState input, kortex_driver::ControllerState &output);
int ToRosData(Kinova::Api::Base::ControllerElementState input, kortex_driver::ControllerElementState &output);
int ToRosData(Kinova::Api::Base::ActionNotification input, kortex_driver::ActionNotification &output);
int ToRosData(Kinova::Api::Base::TrajectoryInfo input, kortex_driver::TrajectoryInfo &output);
int ToRosData(Kinova::Api::Base::ActionExecutionState input, kortex_driver::ActionExecutionState &output);
int ToRosData(Kinova::Api::Base::RobotEventNotification input, kortex_driver::RobotEventNotification &output);
int ToRosData(Kinova::Api::Base::FactoryNotification input, kortex_driver::FactoryNotification &output);
int ToRosData(Kinova::Api::Base::NetworkNotification input, kortex_driver::NetworkNotification &output);
int ToRosData(Kinova::Api::Base::ConfigurationChangeNotificationList input, kortex_driver::ConfigurationChangeNotificationList &output);
int ToRosData(Kinova::Api::Base::MappingInfoNotificationList input, kortex_driver::MappingInfoNotificationList &output);
int ToRosData(Kinova::Api::Base::ControlModeNotificationList input, kortex_driver::ControlModeNotificationList &output);
int ToRosData(Kinova::Api::Base::OperatingModeNotificationList input, kortex_driver::OperatingModeNotificationList &output);
int ToRosData(Kinova::Api::Base::ServoingModeNotificationList input, kortex_driver::ServoingModeNotificationList &output);
int ToRosData(Kinova::Api::Base::SequenceInfoNotificationList input, kortex_driver::SequenceInfoNotificationList &output);
int ToRosData(Kinova::Api::Base::ProtectionZoneNotificationList input, kortex_driver::ProtectionZoneNotificationList &output);
int ToRosData(Kinova::Api::Base::UserNotificationList input, kortex_driver::UserNotificationList &output);
int ToRosData(Kinova::Api::Base::SafetyNotificationList input, kortex_driver::SafetyNotificationList &output);
int ToRosData(Kinova::Api::Base::ControllerNotificationList input, kortex_driver::ControllerNotificationList &output);
int ToRosData(Kinova::Api::Base::ActionNotificationList input, kortex_driver::ActionNotificationList &output);
int ToRosData(Kinova::Api::Base::RobotEventNotificationList input, kortex_driver::RobotEventNotificationList &output);
int ToRosData(Kinova::Api::Base::NetworkNotificationList input, kortex_driver::NetworkNotificationList &output);
int ToRosData(Kinova::Api::Base::MappingHandle input, kortex_driver::MappingHandle &output);
int ToRosData(Kinova::Api::Base::SafetyEvent input, kortex_driver::SafetyEvent &output);
int ToRosData(Kinova::Api::Base::ControllerEvent input, kortex_driver::ControllerEvent &output);
int ToRosData(Kinova::Api::Base::GpioEvent input, kortex_driver::GpioEvent &output);
int ToRosData(Kinova::Api::Base::MapEvent input, kortex_driver::MapEvent &output);
int ToRosData(Kinova::Api::Base::MapElement input, kortex_driver::MapElement &output);
int ToRosData(Kinova::Api::Base::ActivateMapHandle input, kortex_driver::ActivateMapHandle &output);
int ToRosData(Kinova::Api::Base::Map input, kortex_driver::Map &output);
int ToRosData(Kinova::Api::Base::MapHandle input, kortex_driver::MapHandle &output);
int ToRosData(Kinova::Api::Base::MapList input, kortex_driver::MapList &output);
int ToRosData(Kinova::Api::Base::MapGroupHandle input, kortex_driver::MapGroupHandle &output);
int ToRosData(Kinova::Api::Base::MapGroup input, kortex_driver::MapGroup &output);
int ToRosData(Kinova::Api::Base::MapGroupList input, kortex_driver::MapGroupList &output);
int ToRosData(Kinova::Api::Base::Mapping input, kortex_driver::Mapping &output);
int ToRosData(Kinova::Api::Base::MappingList input, kortex_driver::MappingList &output);
int ToRosData(Kinova::Api::Base::TransformationMatrix input, kortex_driver::TransformationMatrix &output);
int ToRosData(Kinova::Api::Base::TransformationRow input, kortex_driver::TransformationRow &output);
int ToRosData(Kinova::Api::Base::Pose input, kortex_driver::Pose &output);
int ToRosData(Kinova::Api::Base::Position input, kortex_driver::Base_Position &output);
int ToRosData(Kinova::Api::Base::Orientation input, kortex_driver::Orientation &output);
int ToRosData(Kinova::Api::Base::CartesianSpeed input, kortex_driver::CartesianSpeed &output);
int ToRosData(Kinova::Api::Base::CartesianTrajectoryConstraint input, kortex_driver::CartesianTrajectoryConstraint &output);
int ToRosData(Kinova::Api::Base::JointTrajectoryConstraint input, kortex_driver::JointTrajectoryConstraint &output);
int ToRosData(Kinova::Api::Base::Wrench input, kortex_driver::Wrench &output);
int ToRosData(Kinova::Api::Base::Twist input, kortex_driver::Twist &output);
int ToRosData(Kinova::Api::Base::Admittance input, kortex_driver::Admittance &output);
int ToRosData(Kinova::Api::Base::ConstrainedPose input, kortex_driver::ConstrainedPose &output);
int ToRosData(Kinova::Api::Base::ConstrainedPosition input, kortex_driver::ConstrainedPosition &output);
int ToRosData(Kinova::Api::Base::ConstrainedOrientation input, kortex_driver::ConstrainedOrientation &output);
int ToRosData(Kinova::Api::Base::WrenchCommand input, kortex_driver::WrenchCommand &output);
int ToRosData(Kinova::Api::Base::TwistCommand input, kortex_driver::TwistCommand &output);
int ToRosData(Kinova::Api::Base::ConstrainedJointAngles input, kortex_driver::ConstrainedJointAngles &output);
int ToRosData(Kinova::Api::Base::ConstrainedJointAngle input, kortex_driver::ConstrainedJointAngle &output);
int ToRosData(Kinova::Api::Base::JointAngles input, kortex_driver::JointAngles &output);
int ToRosData(Kinova::Api::Base::JointAngle input, kortex_driver::JointAngle &output);
int ToRosData(Kinova::Api::Base::JointSpeeds input, kortex_driver::Base_JointSpeeds &output);
int ToRosData(Kinova::Api::Base::JointSpeed input, kortex_driver::JointSpeed &output);
int ToRosData(Kinova::Api::Base::JointTorques input, kortex_driver::JointTorques &output);
int ToRosData(Kinova::Api::Base::JointTorque input, kortex_driver::JointTorque &output);
int ToRosData(Kinova::Api::Base::GripperCommand input, kortex_driver::GripperCommand &output);
int ToRosData(Kinova::Api::Base::GripperRequest input, kortex_driver::GripperRequest &output);
int ToRosData(Kinova::Api::Base::Gripper input, kortex_driver::Gripper &output);
int ToRosData(Kinova::Api::Base::Finger input, kortex_driver::Finger &output);
int ToRosData(Kinova::Api::Base::GpioCommand input, kortex_driver::GpioCommand &output);
int ToRosData(Kinova::Api::Base::SystemTime input, kortex_driver::SystemTime &output);
int ToRosData(Kinova::Api::Base::ControllerConfigurationMode input, kortex_driver::ControllerConfigurationMode &output);
int ToRosData(Kinova::Api::Base::ControllerConfiguration input, kortex_driver::ControllerConfiguration &output);
int ToRosData(Kinova::Api::Base::ControllerConfigurationList input, kortex_driver::ControllerConfigurationList &output);
int ToRosData(Kinova::Api::Base::ActuatorInformation input, kortex_driver::ActuatorInformation &output);
int ToRosData(Kinova::Api::Base::ArmStateInformation input, kortex_driver::ArmStateInformation &output);
int ToRosData(Kinova::Api::Base::ArmStateNotification input, kortex_driver::ArmStateNotification &output);
int ToRosData(Kinova::Api::Base::CapSenseConfig input, kortex_driver::Base_CapSenseConfig &output);
int ToRosData(Kinova::Api::Base::BridgeList input, kortex_driver::BridgeList &output);
int ToRosData(Kinova::Api::Base::BridgeResult input, kortex_driver::BridgeResult &output);
int ToRosData(Kinova::Api::Base::BridgeIdentifier input, kortex_driver::BridgeIdentifier &output);
int ToRosData(Kinova::Api::Base::BridgeConfig input, kortex_driver::BridgeConfig &output);
int ToRosData(Kinova::Api::Base::BridgePortConfig input, kortex_driver::BridgePortConfig &output);
int ToRosData(Kinova::Api::Base::PreComputedJointTrajectory input, kortex_driver::PreComputedJointTrajectory &output);
int ToRosData(Kinova::Api::Base::PreComputedJointTrajectoryElement input, kortex_driver::PreComputedJointTrajectoryElement &output);
int ToRosData(Kinova::Api::Base::TrajectoryErrorElement input, kortex_driver::TrajectoryErrorElement &output);
int ToRosData(Kinova::Api::Base::TrajectoryErrorReport input, kortex_driver::TrajectoryErrorReport &output);
int ToRosData(Kinova::Api::Base::WaypointValidationReport input, kortex_driver::WaypointValidationReport &output);
int ToRosData(Kinova::Api::Base::Waypoint input, kortex_driver::Waypoint &output);
int ToRosData(Kinova::Api::Base::AngularWaypoint input, kortex_driver::AngularWaypoint &output);
int ToRosData(Kinova::Api::Base::CartesianWaypoint input, kortex_driver::CartesianWaypoint &output);
int ToRosData(Kinova::Api::Base::WaypointList input, kortex_driver::WaypointList &output);
int ToRosData(Kinova::Api::Base::KinematicTrajectoryConstraints input, kortex_driver::KinematicTrajectoryConstraints &output);
int ToRosData(Kinova::Api::Base::FirmwareBundleVersions input, kortex_driver::FirmwareBundleVersions &output);
int ToRosData(Kinova::Api::Base::FirmwareComponentVersion input, kortex_driver::FirmwareComponentVersion &output);
int ToRosData(Kinova::Api::Base::IKData input, kortex_driver::IKData &output);

#endif