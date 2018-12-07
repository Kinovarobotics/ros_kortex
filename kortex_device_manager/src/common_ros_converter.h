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
 
#ifndef _KORTEX_CommonROS_CONVERTER_H_
#define _KORTEX_CommonROS_CONVERTER_H_

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

int ToRosData(DeviceHandle input, kortex_device_manager::DeviceHandle &output);
int ToRosData(Empty input, kortex_device_manager::Empty &output);
int ToRosData(NotificationOptions input, kortex_device_manager::NotificationOptions &output);
int ToRosData(SafetyHandle input, kortex_device_manager::SafetyHandle &output);
int ToRosData(NotificationHandle input, kortex_device_manager::NotificationHandle &output);
int ToRosData(SafetyNotification input, kortex_device_manager::SafetyNotification &output);
int ToRosData(Timestamp input, kortex_device_manager::Timestamp &output);
int ToRosData(UserProfileHandle input, kortex_device_manager::UserProfileHandle &output);
int ToRosData(Connection input, kortex_device_manager::Connection &output);

#endif