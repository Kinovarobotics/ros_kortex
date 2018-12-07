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
 
#ifndef _KORTEX_VisionConfigPROTO_CONVERTER_H_
#define _KORTEX_VisionConfigPROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <VisionConfig.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_vision_config_driver/SensorSettings.h"
#include "kortex_vision_config_driver/SensorIdentifier.h"
#include "kortex_vision_config_driver/OptionIdentifier.h"
#include "kortex_vision_config_driver/OptionValue.h"
#include "kortex_vision_config_driver/OptionInformation.h"
#include "kortex_vision_config_driver/SensorFocusAction.h"
#include "kortex_vision_config_driver/VisionNotification.h"
#include "kortex_vision_config_driver/IntrinsicParameters.h"


using namespace Kinova::Api::VisionConfig;

int ToProtoData(kortex_vision_config_driver::SensorSettings intput, SensorSettings *output);
int ToProtoData(kortex_vision_config_driver::SensorIdentifier intput, SensorIdentifier *output);
int ToProtoData(kortex_vision_config_driver::OptionIdentifier intput, OptionIdentifier *output);
int ToProtoData(kortex_vision_config_driver::OptionValue intput, OptionValue *output);
int ToProtoData(kortex_vision_config_driver::OptionInformation intput, OptionInformation *output);
int ToProtoData(kortex_vision_config_driver::SensorFocusAction intput, SensorFocusAction *output);
int ToProtoData(kortex_vision_config_driver::VisionNotification intput, VisionNotification *output);
int ToProtoData(kortex_vision_config_driver::IntrinsicParameters intput, IntrinsicParameters *output);

#endif