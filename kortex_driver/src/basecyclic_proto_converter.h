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
 
#ifndef _KORTEX_BaseCyclicPROTO_CONVERTER_H_
#define _KORTEX_BaseCyclicPROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <BaseCyclic.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_driver/ActuatorCommand.h"
#include "kortex_driver/InterconnectCommand.h"
#include "kortex_driver/ActuatorFeedback.h"
#include "kortex_driver/InterconnectFeedback.h"
#include "kortex_driver/ActuatorCustomData.h"
#include "kortex_driver/InterconnectCustomData.h"
#include "kortex_driver/BaseFeedback.h"
#include "kortex_driver/CustomData.h"
#include "kortex_driver/Command.h"
#include "kortex_driver/Feedback.h"


using namespace Kinova::Api::BaseCyclic;

int ToProtoData(kortex_driver::ActuatorCommand intput, ActuatorCommand *output);
int ToProtoData(kortex_driver::InterconnectCommand intput, InterconnectCommand *output);
int ToProtoData(kortex_driver::ActuatorFeedback intput, ActuatorFeedback *output);
int ToProtoData(kortex_driver::InterconnectFeedback intput, InterconnectFeedback *output);
int ToProtoData(kortex_driver::ActuatorCustomData intput, ActuatorCustomData *output);
int ToProtoData(kortex_driver::InterconnectCustomData intput, InterconnectCustomData *output);
int ToProtoData(kortex_driver::BaseFeedback intput, BaseFeedback *output);
int ToProtoData(kortex_driver::CustomData intput, CustomData *output);
int ToProtoData(kortex_driver::Command intput, Command *output);
int ToProtoData(kortex_driver::Feedback intput, Feedback *output);

#endif