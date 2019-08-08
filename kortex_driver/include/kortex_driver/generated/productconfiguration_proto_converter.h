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
 
#ifndef _KORTEX_PRODUCTCONFIGURATION_PROTO_CONVERTER_H_
#define _KORTEX_PRODUCTCONFIGURATION_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ProductConfiguration.pb.h>

#include "kortex_driver/generated/common_proto_converter.h"
#include "kortex_driver/generated/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/base_proto_converter.h"
#include "kortex_driver/generated/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/basecyclic_proto_converter.h"
#include "kortex_driver/generated/controlconfig_proto_converter.h"
#include "kortex_driver/generated/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/devicemanager_proto_converter.h"
#include "kortex_driver/generated/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/visionconfig_proto_converter.h"


#include "kortex_driver/CompleteProductConfiguration.h"
#include "kortex_driver/ProductConfigurationDegreeOfFreedom.h"
#include "kortex_driver/ProductConfigurationBaseType.h"
#include "kortex_driver/ProductConfigurationEndEffectorType.h"
#include "kortex_driver/ProductConfigurationVisionModuleType.h"
#include "kortex_driver/ProductConfigurationInterfaceModuleType.h"
#include "kortex_driver/ProductConfigurationLaterality.h"
#include "kortex_driver/ProductConfigurationWristType.h"


int ToProtoData(kortex_driver::CompleteProductConfiguration input, Kinova::Api::ProductConfiguration::CompleteProductConfiguration *output);
int ToProtoData(kortex_driver::ProductConfigurationDegreeOfFreedom input, Kinova::Api::ProductConfiguration::ProductConfigurationDegreeOfFreedom *output);
int ToProtoData(kortex_driver::ProductConfigurationBaseType input, Kinova::Api::ProductConfiguration::ProductConfigurationBaseType *output);
int ToProtoData(kortex_driver::ProductConfigurationEndEffectorType input, Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType *output);
int ToProtoData(kortex_driver::ProductConfigurationVisionModuleType input, Kinova::Api::ProductConfiguration::ProductConfigurationVisionModuleType *output);
int ToProtoData(kortex_driver::ProductConfigurationInterfaceModuleType input, Kinova::Api::ProductConfiguration::ProductConfigurationInterfaceModuleType *output);
int ToProtoData(kortex_driver::ProductConfigurationLaterality input, Kinova::Api::ProductConfiguration::ProductConfigurationLaterality *output);
int ToProtoData(kortex_driver::ProductConfigurationWristType input, Kinova::Api::ProductConfiguration::ProductConfigurationWristType *output);

#endif