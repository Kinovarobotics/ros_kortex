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
 
#ifndef _KORTEX_BaseCyclicROS_CONVERTER_H_
#define _KORTEX_BaseCyclicROS_CONVERTER_H_

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

int ToRosData(ActuatorCommand input, kortex_driver::ActuatorCommand &output);
int ToRosData(InterconnectCommand input, kortex_driver::InterconnectCommand &output);
int ToRosData(ActuatorFeedback input, kortex_driver::ActuatorFeedback &output);
int ToRosData(InterconnectFeedback input, kortex_driver::InterconnectFeedback &output);
int ToRosData(ActuatorCustomData input, kortex_driver::ActuatorCustomData &output);
int ToRosData(InterconnectCustomData input, kortex_driver::InterconnectCustomData &output);
int ToRosData(BaseFeedback input, kortex_driver::BaseFeedback &output);
int ToRosData(CustomData input, kortex_driver::CustomData &output);
int ToRosData(Command input, kortex_driver::Command &output);
int ToRosData(Feedback input, kortex_driver::Feedback &output);

#endif