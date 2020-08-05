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
 
#ifndef _KORTEX_ACTUATORCONFIG_ROS_CONVERTER_H_
#define _KORTEX_ACTUATORCONFIG_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ActuatorConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/AxisPosition.h"
#include "kortex_driver/AxisOffsets.h"
#include "kortex_driver/TorqueCalibration.h"
#include "kortex_driver/TorqueOffset.h"
#include "kortex_driver/ActuatorConfig_ControlModeInformation.h"
#include "kortex_driver/ControlLoop.h"
#include "kortex_driver/LoopSelection.h"
#include "kortex_driver/VectorDriveParameters.h"
#include "kortex_driver/EncoderDerivativeParameters.h"
#include "kortex_driver/ControlLoopParameters.h"
#include "kortex_driver/FrequencyResponse.h"
#include "kortex_driver/StepResponse.h"
#include "kortex_driver/RampResponse.h"
#include "kortex_driver/CustomDataSelection.h"
#include "kortex_driver/CommandModeInformation.h"
#include "kortex_driver/Servoing.h"
#include "kortex_driver/PositionCommand.h"
#include "kortex_driver/CoggingFeedforwardModeInformation.h"


int ToRosData(Kinova::Api::ActuatorConfig::AxisPosition input, kortex_driver::AxisPosition &output);
int ToRosData(Kinova::Api::ActuatorConfig::AxisOffsets input, kortex_driver::AxisOffsets &output);
int ToRosData(Kinova::Api::ActuatorConfig::TorqueCalibration input, kortex_driver::TorqueCalibration &output);
int ToRosData(Kinova::Api::ActuatorConfig::TorqueOffset input, kortex_driver::TorqueOffset &output);
int ToRosData(Kinova::Api::ActuatorConfig::ControlModeInformation input, kortex_driver::ActuatorConfig_ControlModeInformation &output);
int ToRosData(Kinova::Api::ActuatorConfig::ControlLoop input, kortex_driver::ControlLoop &output);
int ToRosData(Kinova::Api::ActuatorConfig::LoopSelection input, kortex_driver::LoopSelection &output);
int ToRosData(Kinova::Api::ActuatorConfig::VectorDriveParameters input, kortex_driver::VectorDriveParameters &output);
int ToRosData(Kinova::Api::ActuatorConfig::EncoderDerivativeParameters input, kortex_driver::EncoderDerivativeParameters &output);
int ToRosData(Kinova::Api::ActuatorConfig::ControlLoopParameters input, kortex_driver::ControlLoopParameters &output);
int ToRosData(Kinova::Api::ActuatorConfig::FrequencyResponse input, kortex_driver::FrequencyResponse &output);
int ToRosData(Kinova::Api::ActuatorConfig::StepResponse input, kortex_driver::StepResponse &output);
int ToRosData(Kinova::Api::ActuatorConfig::RampResponse input, kortex_driver::RampResponse &output);
int ToRosData(Kinova::Api::ActuatorConfig::CustomDataSelection input, kortex_driver::CustomDataSelection &output);
int ToRosData(Kinova::Api::ActuatorConfig::CommandModeInformation input, kortex_driver::CommandModeInformation &output);
int ToRosData(Kinova::Api::ActuatorConfig::Servoing input, kortex_driver::Servoing &output);
int ToRosData(Kinova::Api::ActuatorConfig::PositionCommand input, kortex_driver::PositionCommand &output);
int ToRosData(Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation input, kortex_driver::CoggingFeedforwardModeInformation &output);

#endif