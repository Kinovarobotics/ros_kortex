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
 
#ifndef _KORTEX_CONTROLCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_CONTROLCONFIG_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::GravityVector input, Kinova::Api::ControlConfig::GravityVector *output);
int ToProtoData(kortex_driver::ControlConfig_Position input, Kinova::Api::ControlConfig::Position *output);
int ToProtoData(kortex_driver::PayloadInformation input, Kinova::Api::ControlConfig::PayloadInformation *output);
int ToProtoData(kortex_driver::CartesianTransform input, Kinova::Api::ControlConfig::CartesianTransform *output);
int ToProtoData(kortex_driver::ToolConfiguration input, Kinova::Api::ControlConfig::ToolConfiguration *output);
int ToProtoData(kortex_driver::ControlConfigurationNotification input, Kinova::Api::ControlConfig::ControlConfigurationNotification *output);
int ToProtoData(kortex_driver::CartesianReferenceFrameInfo input, Kinova::Api::ControlConfig::CartesianReferenceFrameInfo *output);
int ToProtoData(kortex_driver::TwistLinearSoftLimit input, Kinova::Api::ControlConfig::TwistLinearSoftLimit *output);
int ToProtoData(kortex_driver::TwistAngularSoftLimit input, Kinova::Api::ControlConfig::TwistAngularSoftLimit *output);
int ToProtoData(kortex_driver::JointSpeedSoftLimits input, Kinova::Api::ControlConfig::JointSpeedSoftLimits *output);
int ToProtoData(kortex_driver::JointAccelerationSoftLimits input, Kinova::Api::ControlConfig::JointAccelerationSoftLimits *output);
int ToProtoData(kortex_driver::KinematicLimits input, Kinova::Api::ControlConfig::KinematicLimits *output);
int ToProtoData(kortex_driver::KinematicLimitsList input, Kinova::Api::ControlConfig::KinematicLimitsList *output);
int ToProtoData(kortex_driver::DesiredSpeeds input, Kinova::Api::ControlConfig::DesiredSpeeds *output);
int ToProtoData(kortex_driver::LinearTwist input, Kinova::Api::ControlConfig::LinearTwist *output);
int ToProtoData(kortex_driver::AngularTwist input, Kinova::Api::ControlConfig::AngularTwist *output);
int ToProtoData(kortex_driver::ControlConfig_JointSpeeds input, Kinova::Api::ControlConfig::JointSpeeds *output);
int ToProtoData(kortex_driver::ControlConfig_ControlModeInformation input, Kinova::Api::ControlConfig::ControlModeInformation *output);
int ToProtoData(kortex_driver::ControlConfig_ControlModeNotification input, Kinova::Api::ControlConfig::ControlModeNotification *output);

#endif