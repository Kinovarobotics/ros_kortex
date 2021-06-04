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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_INTERCONNECTCONFIG_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <InterconnectConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
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
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::EthernetDeviceIdentification input, Kinova::Api::InterconnectConfig::EthernetDeviceIdentification *output);
int ToProtoData(kortex_driver::EthernetConfiguration input, Kinova::Api::InterconnectConfig::EthernetConfiguration *output);
int ToProtoData(kortex_driver::GPIOIdentification input, Kinova::Api::InterconnectConfig::GPIOIdentification *output);
int ToProtoData(kortex_driver::InterconnectConfig_GPIOConfiguration input, Kinova::Api::InterconnectConfig::GPIOConfiguration *output);
int ToProtoData(kortex_driver::GPIOState input, Kinova::Api::InterconnectConfig::GPIOState *output);
int ToProtoData(kortex_driver::I2CDeviceIdentification input, Kinova::Api::InterconnectConfig::I2CDeviceIdentification *output);
int ToProtoData(kortex_driver::I2CConfiguration input, Kinova::Api::InterconnectConfig::I2CConfiguration *output);
int ToProtoData(kortex_driver::I2CReadParameter input, Kinova::Api::InterconnectConfig::I2CReadParameter *output);
int ToProtoData(kortex_driver::I2CReadRegisterParameter input, Kinova::Api::InterconnectConfig::I2CReadRegisterParameter *output);
int ToProtoData(kortex_driver::I2CWriteParameter input, Kinova::Api::InterconnectConfig::I2CWriteParameter *output);
int ToProtoData(kortex_driver::I2CWriteRegisterParameter input, Kinova::Api::InterconnectConfig::I2CWriteRegisterParameter *output);
int ToProtoData(kortex_driver::I2CData input, Kinova::Api::InterconnectConfig::I2CData *output);

#endif