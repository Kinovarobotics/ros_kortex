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
 
#ifndef _KORTEX_ActuatorCyclicROS_CONVERTER_H_
#define _KORTEX_ActuatorCyclicROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <ActuatorCyclic.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_actuator_driver/MessageId.h"
#include "kortex_actuator_driver/Command.h"
#include "kortex_actuator_driver/Feedback.h"
#include "kortex_actuator_driver/CustomData.h"


using namespace Kinova::Api::ActuatorCyclic;

int ToRosData(MessageId input, kortex_actuator_driver::MessageId &output);
int ToRosData(Command input, kortex_actuator_driver::Command &output);
int ToRosData(Feedback input, kortex_actuator_driver::Feedback &output);
int ToRosData(CustomData input, kortex_actuator_driver::CustomData &output);

#endif