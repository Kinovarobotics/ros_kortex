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
 
#ifndef _KORTEX_CONTROLCONFIG_ROS_CONVERTER_H_
#define _KORTEX_CONTROLCONFIG_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>

#include "kortex_driver/generated/common_ros_converter.h"
#include "kortex_driver/generated/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/base_ros_converter.h"
#include "kortex_driver/generated/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/basecyclic_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/devicemanager_ros_converter.h"
#include "kortex_driver/generated/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/visionconfig_ros_converter.h"


#include "kortex_driver/GravityVector.h"
#include "kortex_driver/ControlConfig_Position.h"
#include "kortex_driver/PayloadInformation.h"
#include "kortex_driver/CartesianTransform.h"
#include "kortex_driver/ToolConfiguration.h"
#include "kortex_driver/ControlConfigurationNotification.h"
#include "kortex_driver/CartesianReferenceFrameInfo.h"


int ToRosData(Kinova::Api::ControlConfig::GravityVector input, kortex_driver::GravityVector &output);
int ToRosData(Kinova::Api::ControlConfig::Position input, kortex_driver::ControlConfig_Position &output);
int ToRosData(Kinova::Api::ControlConfig::PayloadInformation input, kortex_driver::PayloadInformation &output);
int ToRosData(Kinova::Api::ControlConfig::CartesianTransform input, kortex_driver::CartesianTransform &output);
int ToRosData(Kinova::Api::ControlConfig::ToolConfiguration input, kortex_driver::ToolConfiguration &output);
int ToRosData(Kinova::Api::ControlConfig::ControlConfigurationNotification input, kortex_driver::ControlConfigurationNotification &output);
int ToRosData(Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input, kortex_driver::CartesianReferenceFrameInfo &output);

#endif