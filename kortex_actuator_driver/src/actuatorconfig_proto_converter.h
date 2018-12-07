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
 
#ifndef _KORTEX_ActuatorConfigPROTO_CONVERTER_H_
#define _KORTEX_ActuatorConfigPROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <ActuatorConfig.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_actuator_driver/AxisPosition.h"
#include "kortex_actuator_driver/AxisOffsets.h"
#include "kortex_actuator_driver/TorqueCalibration.h"
#include "kortex_actuator_driver/TorqueOffset.h"
#include "kortex_actuator_driver/ControlModeInformation.h"
#include "kortex_actuator_driver/ControlLoop.h"
#include "kortex_actuator_driver/LoopSelection.h"
#include "kortex_actuator_driver/VectorDriveParameters.h"
#include "kortex_actuator_driver/EncoderDerivativeParameters.h"
#include "kortex_actuator_driver/ControlLoopParameters.h"
#include "kortex_actuator_driver/FrequencyResponse.h"
#include "kortex_actuator_driver/StepResponse.h"
#include "kortex_actuator_driver/RampResponse.h"
#include "kortex_actuator_driver/CustomDataSelection.h"
#include "kortex_actuator_driver/CommandModeInformation.h"
#include "kortex_actuator_driver/Servoing.h"
#include "kortex_actuator_driver/PositionCommand.h"


using namespace Kinova::Api::ActuatorConfig;

int ToProtoData(kortex_actuator_driver::AxisPosition intput, AxisPosition *output);
int ToProtoData(kortex_actuator_driver::AxisOffsets intput, AxisOffsets *output);
int ToProtoData(kortex_actuator_driver::TorqueCalibration intput, TorqueCalibration *output);
int ToProtoData(kortex_actuator_driver::TorqueOffset intput, TorqueOffset *output);
int ToProtoData(kortex_actuator_driver::ControlModeInformation intput, ControlModeInformation *output);
int ToProtoData(kortex_actuator_driver::ControlLoop intput, ControlLoop *output);
int ToProtoData(kortex_actuator_driver::LoopSelection intput, LoopSelection *output);
int ToProtoData(kortex_actuator_driver::VectorDriveParameters intput, VectorDriveParameters *output);
int ToProtoData(kortex_actuator_driver::EncoderDerivativeParameters intput, EncoderDerivativeParameters *output);
int ToProtoData(kortex_actuator_driver::ControlLoopParameters intput, ControlLoopParameters *output);
int ToProtoData(kortex_actuator_driver::FrequencyResponse intput, FrequencyResponse *output);
int ToProtoData(kortex_actuator_driver::StepResponse intput, StepResponse *output);
int ToProtoData(kortex_actuator_driver::RampResponse intput, RampResponse *output);
int ToProtoData(kortex_actuator_driver::CustomDataSelection intput, CustomDataSelection *output);
int ToProtoData(kortex_actuator_driver::CommandModeInformation intput, CommandModeInformation *output);
int ToProtoData(kortex_actuator_driver::Servoing intput, Servoing *output);
int ToProtoData(kortex_actuator_driver::PositionCommand intput, PositionCommand *output);

#endif