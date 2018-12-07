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
 
#ifndef _KORTEX_ActuatorConfigROS_CONVERTER_H_
#define _KORTEX_ActuatorConfigROS_CONVERTER_H_

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

int ToRosData(AxisPosition input, kortex_actuator_driver::AxisPosition &output);
int ToRosData(AxisOffsets input, kortex_actuator_driver::AxisOffsets &output);
int ToRosData(TorqueCalibration input, kortex_actuator_driver::TorqueCalibration &output);
int ToRosData(TorqueOffset input, kortex_actuator_driver::TorqueOffset &output);
int ToRosData(ControlModeInformation input, kortex_actuator_driver::ControlModeInformation &output);
int ToRosData(ControlLoop input, kortex_actuator_driver::ControlLoop &output);
int ToRosData(LoopSelection input, kortex_actuator_driver::LoopSelection &output);
int ToRosData(VectorDriveParameters input, kortex_actuator_driver::VectorDriveParameters &output);
int ToRosData(EncoderDerivativeParameters input, kortex_actuator_driver::EncoderDerivativeParameters &output);
int ToRosData(ControlLoopParameters input, kortex_actuator_driver::ControlLoopParameters &output);
int ToRosData(FrequencyResponse input, kortex_actuator_driver::FrequencyResponse &output);
int ToRosData(StepResponse input, kortex_actuator_driver::StepResponse &output);
int ToRosData(RampResponse input, kortex_actuator_driver::RampResponse &output);
int ToRosData(CustomDataSelection input, kortex_actuator_driver::CustomDataSelection &output);
int ToRosData(CommandModeInformation input, kortex_actuator_driver::CommandModeInformation &output);
int ToRosData(Servoing input, kortex_actuator_driver::Servoing &output);
int ToRosData(PositionCommand input, kortex_actuator_driver::PositionCommand &output);

#endif