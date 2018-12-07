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
 
#ifndef _KORTEX_CommonPROTO_CONVERTER_H_
#define _KORTEX_CommonPROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <Common.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_driver/DeviceHandle.h"
#include "kortex_driver/Empty.h"
#include "kortex_driver/NotificationOptions.h"
#include "kortex_driver/SafetyHandle.h"
#include "kortex_driver/NotificationHandle.h"
#include "kortex_driver/SafetyNotification.h"
#include "kortex_driver/Timestamp.h"
#include "kortex_driver/UserProfileHandle.h"
#include "kortex_driver/Connection.h"


using namespace Kinova::Api::Common;

int ToProtoData(kortex_driver::DeviceHandle intput, DeviceHandle *output);
int ToProtoData(kortex_driver::Empty intput, Empty *output);
int ToProtoData(kortex_driver::NotificationOptions intput, NotificationOptions *output);
int ToProtoData(kortex_driver::SafetyHandle intput, SafetyHandle *output);
int ToProtoData(kortex_driver::NotificationHandle intput, NotificationHandle *output);
int ToProtoData(kortex_driver::SafetyNotification intput, SafetyNotification *output);
int ToProtoData(kortex_driver::Timestamp intput, Timestamp *output);
int ToProtoData(kortex_driver::UserProfileHandle intput, UserProfileHandle *output);
int ToProtoData(kortex_driver::Connection intput, Connection *output);

#endif