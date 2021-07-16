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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_ROS_CONVERTER_H_
#define _KORTEX_INTERCONNECTCONFIG_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <InterconnectConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
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
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/EthernetDeviceIdentification.h"
#include "kortex_driver/EthernetConfiguration.h"
#include "kortex_driver/GPIOIdentification.h"
#include "kortex_driver/InterconnectConfig_GPIOConfiguration.h"
#include "kortex_driver/GPIOState.h"
#include "kortex_driver/I2CDeviceIdentification.h"
#include "kortex_driver/I2CConfiguration.h"
#include "kortex_driver/I2CReadParameter.h"
#include "kortex_driver/I2CReadRegisterParameter.h"
#include "kortex_driver/I2CWriteParameter.h"
#include "kortex_driver/I2CWriteRegisterParameter.h"
#include "kortex_driver/I2CData.h"


int ToRosData(Kinova::Api::InterconnectConfig::EthernetDeviceIdentification input, kortex_driver::EthernetDeviceIdentification &output);
int ToRosData(Kinova::Api::InterconnectConfig::EthernetConfiguration input, kortex_driver::EthernetConfiguration &output);
int ToRosData(Kinova::Api::InterconnectConfig::GPIOIdentification input, kortex_driver::GPIOIdentification &output);
int ToRosData(Kinova::Api::InterconnectConfig::GPIOConfiguration input, kortex_driver::InterconnectConfig_GPIOConfiguration &output);
int ToRosData(Kinova::Api::InterconnectConfig::GPIOState input, kortex_driver::GPIOState &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CDeviceIdentification input, kortex_driver::I2CDeviceIdentification &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CConfiguration input, kortex_driver::I2CConfiguration &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CReadParameter input, kortex_driver::I2CReadParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CReadRegisterParameter input, kortex_driver::I2CReadRegisterParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CWriteParameter input, kortex_driver::I2CWriteParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter input, kortex_driver::I2CWriteRegisterParameter &output);
int ToRosData(Kinova::Api::InterconnectConfig::I2CData input, kortex_driver::I2CData &output);

#endif