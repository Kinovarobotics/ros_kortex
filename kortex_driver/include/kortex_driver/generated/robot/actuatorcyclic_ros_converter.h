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
 
#ifndef _KORTEX_ACTUATORCYCLIC_ROS_CONVERTER_H_
#define _KORTEX_ACTUATORCYCLIC_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ActuatorCyclic.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
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


#include "kortex_driver/ActuatorCyclic_MessageId.h"
#include "kortex_driver/ActuatorCyclic_Command.h"
#include "kortex_driver/ActuatorCyclic_Feedback.h"
#include "kortex_driver/ActuatorCyclic_CustomData.h"


int ToRosData(Kinova::Api::ActuatorCyclic::MessageId input, kortex_driver::ActuatorCyclic_MessageId &output);
int ToRosData(Kinova::Api::ActuatorCyclic::Command input, kortex_driver::ActuatorCyclic_Command &output);
int ToRosData(Kinova::Api::ActuatorCyclic::Feedback input, kortex_driver::ActuatorCyclic_Feedback &output);
int ToRosData(Kinova::Api::ActuatorCyclic::CustomData input, kortex_driver::ActuatorCyclic_CustomData &output);

#endif