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
 
#ifndef _KORTEX_ACTUATORCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_ACTUATORCONFIG_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ActuatorConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::AxisPosition input, Kinova::Api::ActuatorConfig::AxisPosition *output);
int ToProtoData(kortex_driver::AxisOffsets input, Kinova::Api::ActuatorConfig::AxisOffsets *output);
int ToProtoData(kortex_driver::TorqueCalibration input, Kinova::Api::ActuatorConfig::TorqueCalibration *output);
int ToProtoData(kortex_driver::TorqueOffset input, Kinova::Api::ActuatorConfig::TorqueOffset *output);
int ToProtoData(kortex_driver::ActuatorConfig_ControlModeInformation input, Kinova::Api::ActuatorConfig::ControlModeInformation *output);
int ToProtoData(kortex_driver::ControlLoop input, Kinova::Api::ActuatorConfig::ControlLoop *output);
int ToProtoData(kortex_driver::LoopSelection input, Kinova::Api::ActuatorConfig::LoopSelection *output);
int ToProtoData(kortex_driver::VectorDriveParameters input, Kinova::Api::ActuatorConfig::VectorDriveParameters *output);
int ToProtoData(kortex_driver::EncoderDerivativeParameters input, Kinova::Api::ActuatorConfig::EncoderDerivativeParameters *output);
int ToProtoData(kortex_driver::ControlLoopParameters input, Kinova::Api::ActuatorConfig::ControlLoopParameters *output);
int ToProtoData(kortex_driver::FrequencyResponse input, Kinova::Api::ActuatorConfig::FrequencyResponse *output);
int ToProtoData(kortex_driver::StepResponse input, Kinova::Api::ActuatorConfig::StepResponse *output);
int ToProtoData(kortex_driver::RampResponse input, Kinova::Api::ActuatorConfig::RampResponse *output);
int ToProtoData(kortex_driver::CustomDataSelection input, Kinova::Api::ActuatorConfig::CustomDataSelection *output);
int ToProtoData(kortex_driver::CommandModeInformation input, Kinova::Api::ActuatorConfig::CommandModeInformation *output);
int ToProtoData(kortex_driver::Servoing input, Kinova::Api::ActuatorConfig::Servoing *output);
int ToProtoData(kortex_driver::PositionCommand input, Kinova::Api::ActuatorConfig::PositionCommand *output);
int ToProtoData(kortex_driver::CoggingFeedforwardModeInformation input, Kinova::Api::ActuatorConfig::CoggingFeedforwardModeInformation *output);

#endif