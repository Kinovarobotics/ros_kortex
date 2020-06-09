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
 
#ifndef _KORTEX_COMMON_ROS_CONVERTER_H_
#define _KORTEX_COMMON_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Common.pb.h>

#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


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


int ToRosData(Kinova::Api::Common::DeviceHandle input, kortex_driver::DeviceHandle &output);
int ToRosData(Kinova::Api::Common::Empty input, kortex_driver::Empty &output);
int ToRosData(Kinova::Api::Common::NotificationOptions input, kortex_driver::NotificationOptions &output);
int ToRosData(Kinova::Api::Common::SafetyHandle input, kortex_driver::SafetyHandle &output);
int ToRosData(Kinova::Api::Common::NotificationHandle input, kortex_driver::NotificationHandle &output);
int ToRosData(Kinova::Api::Common::SafetyNotification input, kortex_driver::SafetyNotification &output);
int ToRosData(Kinova::Api::Common::Timestamp input, kortex_driver::Timestamp &output);
int ToRosData(Kinova::Api::Common::UserProfileHandle input, kortex_driver::UserProfileHandle &output);
int ToRosData(Kinova::Api::Common::Connection input, kortex_driver::Connection &output);
int ToRosData(Kinova::Api::Common::UARTConfiguration input, kortex_driver::UARTConfiguration &output);
int ToRosData(Kinova::Api::Common::UARTDeviceIdentification input, kortex_driver::UARTDeviceIdentification &output);
int ToRosData(Kinova::Api::Common::CountryCode input, kortex_driver::CountryCode &output);

#endif