/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
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
 
#ifndef _KORTEX_COMMON_PROTO_CONVERTER_H_
#define _KORTEX_COMMON_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Common.pb.h>

#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/DeviceHandle.h"
#include "kortex_driver/Empty.h"
#include "kortex_driver/NotificationOptions.h"
#include "kortex_driver/SafetyHandle.h"
#include "kortex_driver/NotificationHandle.h"
#include "kortex_driver/SafetyNotification.h"
#include "kortex_driver/Timestamp.h"
#include "kortex_driver/UserProfileHandle.h"
#include "kortex_driver/Connection.h"
#include "kortex_driver/UARTConfiguration.h"
#include "kortex_driver/UARTDeviceIdentification.h"
#include "kortex_driver/CountryCode.h"


int ToProtoData(kortex_driver::DeviceHandle input, Kinova::Api::Common::DeviceHandle *output);
int ToProtoData(kortex_driver::Empty input, Kinova::Api::Common::Empty *output);
int ToProtoData(kortex_driver::NotificationOptions input, Kinova::Api::Common::NotificationOptions *output);
int ToProtoData(kortex_driver::SafetyHandle input, Kinova::Api::Common::SafetyHandle *output);
int ToProtoData(kortex_driver::NotificationHandle input, Kinova::Api::Common::NotificationHandle *output);
int ToProtoData(kortex_driver::SafetyNotification input, Kinova::Api::Common::SafetyNotification *output);
int ToProtoData(kortex_driver::Timestamp input, Kinova::Api::Common::Timestamp *output);
int ToProtoData(kortex_driver::UserProfileHandle input, Kinova::Api::Common::UserProfileHandle *output);
int ToProtoData(kortex_driver::Connection input, Kinova::Api::Common::Connection *output);
int ToProtoData(kortex_driver::UARTConfiguration input, Kinova::Api::Common::UARTConfiguration *output);
int ToProtoData(kortex_driver::UARTDeviceIdentification input, Kinova::Api::Common::UARTDeviceIdentification *output);
int ToProtoData(kortex_driver::CountryCode input, Kinova::Api::Common::CountryCode *output);

#endif