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
 
#ifndef _KORTEX_PRODUCTCONFIGURATION_ROS_CONVERTER_H_
#define _KORTEX_PRODUCTCONFIGURATION_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ProductConfiguration.pb.h>

#include "kortex_driver/generated/common_ros_converter.h"
#include "kortex_driver/generated/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/base_ros_converter.h"
#include "kortex_driver/generated/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/basecyclic_ros_converter.h"
#include "kortex_driver/generated/controlconfig_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/devicemanager_ros_converter.h"
#include "kortex_driver/generated/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/visionconfig_ros_converter.h"


#include "kortex_driver/CompleteProductConfiguration.h"
#include "kortex_driver/ProductConfigurationDegreeOfFreedom.h"
#include "kortex_driver/ProductConfigurationBaseType.h"
#include "kortex_driver/ProductConfigurationEndEffectorType.h"
#include "kortex_driver/ProductConfigurationVisionModuleType.h"
#include "kortex_driver/ProductConfigurationInterfaceModuleType.h"
#include "kortex_driver/ProductConfigurationLaterality.h"
#include "kortex_driver/ProductConfigurationWristType.h"


int ToRosData(Kinova::Api::ProductConfiguration::CompleteProductConfiguration input, kortex_driver::CompleteProductConfiguration &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationDegreeOfFreedom input, kortex_driver::ProductConfigurationDegreeOfFreedom &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationBaseType input, kortex_driver::ProductConfigurationBaseType &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType input, kortex_driver::ProductConfigurationEndEffectorType &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationVisionModuleType input, kortex_driver::ProductConfigurationVisionModuleType &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationInterfaceModuleType input, kortex_driver::ProductConfigurationInterfaceModuleType &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationLaterality input, kortex_driver::ProductConfigurationLaterality &output);
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationWristType input, kortex_driver::ProductConfigurationWristType &output);

#endif