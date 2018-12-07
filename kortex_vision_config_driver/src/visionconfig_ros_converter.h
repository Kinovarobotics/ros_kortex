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
 
#ifndef _KORTEX_VisionConfigROS_CONVERTER_H_
#define _KORTEX_VisionConfigROS_CONVERTER_H_

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

int ToRosData(SensorSettings input, kortex_vision_config_driver::SensorSettings &output);
int ToRosData(SensorIdentifier input, kortex_vision_config_driver::SensorIdentifier &output);
int ToRosData(OptionIdentifier input, kortex_vision_config_driver::OptionIdentifier &output);
int ToRosData(OptionValue input, kortex_vision_config_driver::OptionValue &output);
int ToRosData(OptionInformation input, kortex_vision_config_driver::OptionInformation &output);
int ToRosData(SensorFocusAction input, kortex_vision_config_driver::SensorFocusAction &output);
int ToRosData(VisionNotification input, kortex_vision_config_driver::VisionNotification &output);
int ToRosData(IntrinsicParameters input, kortex_vision_config_driver::IntrinsicParameters &output);

#endif