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
 
#ifndef _KORTEX_BASECYCLIC_ROS_CONVERTER_H_
#define _KORTEX_BASECYCLIC_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <BaseCyclic.pb.h>

#include "kortex_driver/generated/common_ros_converter.h"
#include "kortex_driver/generated/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/base_ros_converter.h"
#include "kortex_driver/generated/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/controlconfig_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/devicemanager_ros_converter.h"
#include "kortex_driver/generated/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/visionconfig_ros_converter.h"


#include "kortex_driver/ActuatorCommand.h"
#include "kortex_driver/ActuatorFeedback.h"
#include "kortex_driver/ActuatorCustomData.h"
#include "kortex_driver/BaseFeedback.h"
#include "kortex_driver/BaseCyclic_CustomData.h"
#include "kortex_driver/BaseCyclic_Command.h"
#include "kortex_driver/BaseCyclic_Feedback.h"


int ToRosData(Kinova::Api::BaseCyclic::ActuatorCommand input, kortex_driver::ActuatorCommand &output);
int ToRosData(Kinova::Api::BaseCyclic::ActuatorFeedback input, kortex_driver::ActuatorFeedback &output);
int ToRosData(Kinova::Api::BaseCyclic::ActuatorCustomData input, kortex_driver::ActuatorCustomData &output);
int ToRosData(Kinova::Api::BaseCyclic::BaseFeedback input, kortex_driver::BaseFeedback &output);
int ToRosData(Kinova::Api::BaseCyclic::CustomData input, kortex_driver::BaseCyclic_CustomData &output);
int ToRosData(Kinova::Api::BaseCyclic::Command input, kortex_driver::BaseCyclic_Command &output);
int ToRosData(Kinova::Api::BaseCyclic::Feedback input, kortex_driver::BaseCyclic_Feedback &output);

#endif