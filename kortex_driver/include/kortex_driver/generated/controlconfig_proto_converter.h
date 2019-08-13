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
 
#ifndef _KORTEX_CONTROLCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_CONTROLCONFIG_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>

#include "kortex_driver/generated/common_proto_converter.h"
#include "kortex_driver/generated/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/base_proto_converter.h"
#include "kortex_driver/generated/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/basecyclic_proto_converter.h"
#include "kortex_driver/generated/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/devicemanager_proto_converter.h"
#include "kortex_driver/generated/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/visionconfig_proto_converter.h"


#include "kortex_driver/GravityVector.h"
#include "kortex_driver/ControlConfig_Position.h"
#include "kortex_driver/PayloadInformation.h"
#include "kortex_driver/CartesianTransform.h"
#include "kortex_driver/ToolConfiguration.h"
#include "kortex_driver/ControlConfigurationNotification.h"
#include "kortex_driver/CartesianReferenceFrameInfo.h"


int ToProtoData(kortex_driver::GravityVector input, Kinova::Api::ControlConfig::GravityVector *output);
int ToProtoData(kortex_driver::ControlConfig_Position input, Kinova::Api::ControlConfig::Position *output);
int ToProtoData(kortex_driver::PayloadInformation input, Kinova::Api::ControlConfig::PayloadInformation *output);
int ToProtoData(kortex_driver::CartesianTransform input, Kinova::Api::ControlConfig::CartesianTransform *output);
int ToProtoData(kortex_driver::ToolConfiguration input, Kinova::Api::ControlConfig::ToolConfiguration *output);
int ToProtoData(kortex_driver::ControlConfigurationNotification input, Kinova::Api::ControlConfig::ControlConfigurationNotification *output);
int ToProtoData(kortex_driver::CartesianReferenceFrameInfo input, Kinova::Api::ControlConfig::CartesianReferenceFrameInfo *output);

#endif