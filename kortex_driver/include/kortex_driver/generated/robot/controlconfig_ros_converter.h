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
 
#ifndef _KORTEX_CONTROLCONFIG_ROS_CONVERTER_H_
#define _KORTEX_CONTROLCONFIG_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/GravityVector.h"
#include "kortex_driver/ControlConfig_Position.h"
#include "kortex_driver/PayloadInformation.h"
#include "kortex_driver/CartesianTransform.h"
#include "kortex_driver/ToolConfiguration.h"
#include "kortex_driver/ControlConfigurationNotification.h"
#include "kortex_driver/CartesianReferenceFrameInfo.h"
#include "kortex_driver/TwistLinearSoftLimit.h"
#include "kortex_driver/TwistAngularSoftLimit.h"
#include "kortex_driver/JointSpeedSoftLimits.h"
#include "kortex_driver/JointAccelerationSoftLimits.h"
#include "kortex_driver/KinematicLimits.h"
#include "kortex_driver/KinematicLimitsList.h"
#include "kortex_driver/DesiredSpeeds.h"
#include "kortex_driver/LinearTwist.h"
#include "kortex_driver/AngularTwist.h"
#include "kortex_driver/ControlConfig_JointSpeeds.h"
#include "kortex_driver/ControlConfig_ControlModeInformation.h"
#include "kortex_driver/ControlConfig_ControlModeNotification.h"


int ToRosData(Kinova::Api::ControlConfig::GravityVector input, kortex_driver::GravityVector &output);
int ToRosData(Kinova::Api::ControlConfig::Position input, kortex_driver::ControlConfig_Position &output);
int ToRosData(Kinova::Api::ControlConfig::PayloadInformation input, kortex_driver::PayloadInformation &output);
int ToRosData(Kinova::Api::ControlConfig::CartesianTransform input, kortex_driver::CartesianTransform &output);
int ToRosData(Kinova::Api::ControlConfig::ToolConfiguration input, kortex_driver::ToolConfiguration &output);
int ToRosData(Kinova::Api::ControlConfig::ControlConfigurationNotification input, kortex_driver::ControlConfigurationNotification &output);
int ToRosData(Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input, kortex_driver::CartesianReferenceFrameInfo &output);
int ToRosData(Kinova::Api::ControlConfig::TwistLinearSoftLimit input, kortex_driver::TwistLinearSoftLimit &output);
int ToRosData(Kinova::Api::ControlConfig::TwistAngularSoftLimit input, kortex_driver::TwistAngularSoftLimit &output);
int ToRosData(Kinova::Api::ControlConfig::JointSpeedSoftLimits input, kortex_driver::JointSpeedSoftLimits &output);
int ToRosData(Kinova::Api::ControlConfig::JointAccelerationSoftLimits input, kortex_driver::JointAccelerationSoftLimits &output);
int ToRosData(Kinova::Api::ControlConfig::KinematicLimits input, kortex_driver::KinematicLimits &output);
int ToRosData(Kinova::Api::ControlConfig::KinematicLimitsList input, kortex_driver::KinematicLimitsList &output);
int ToRosData(Kinova::Api::ControlConfig::DesiredSpeeds input, kortex_driver::DesiredSpeeds &output);
int ToRosData(Kinova::Api::ControlConfig::LinearTwist input, kortex_driver::LinearTwist &output);
int ToRosData(Kinova::Api::ControlConfig::AngularTwist input, kortex_driver::AngularTwist &output);
int ToRosData(Kinova::Api::ControlConfig::JointSpeeds input, kortex_driver::ControlConfig_JointSpeeds &output);
int ToRosData(Kinova::Api::ControlConfig::ControlModeInformation input, kortex_driver::ControlConfig_ControlModeInformation &output);
int ToRosData(Kinova::Api::ControlConfig::ControlModeNotification input, kortex_driver::ControlConfig_ControlModeNotification &output);

#endif