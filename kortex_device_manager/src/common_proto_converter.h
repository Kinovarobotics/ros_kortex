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

#include "kortex_device_manager/DeviceHandle.h"
#include "kortex_device_manager/Empty.h"
#include "kortex_device_manager/NotificationOptions.h"
#include "kortex_device_manager/SafetyHandle.h"
#include "kortex_device_manager/NotificationHandle.h"
#include "kortex_device_manager/SafetyNotification.h"
#include "kortex_device_manager/Timestamp.h"
#include "kortex_device_manager/UserProfileHandle.h"
#include "kortex_device_manager/Connection.h"


using namespace Kinova::Api::Common;

int ToProtoData(kortex_device_manager::DeviceHandle intput, DeviceHandle *output);
int ToProtoData(kortex_device_manager::Empty intput, Empty *output);
int ToProtoData(kortex_device_manager::NotificationOptions intput, NotificationOptions *output);
int ToProtoData(kortex_device_manager::SafetyHandle intput, SafetyHandle *output);
int ToProtoData(kortex_device_manager::NotificationHandle intput, NotificationHandle *output);
int ToProtoData(kortex_device_manager::SafetyNotification intput, SafetyNotification *output);
int ToProtoData(kortex_device_manager::Timestamp intput, Timestamp *output);
int ToProtoData(kortex_device_manager::UserProfileHandle intput, UserProfileHandle *output);
int ToProtoData(kortex_device_manager::Connection intput, Connection *output);

#endif