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
 
#ifndef _KORTEX_GRIPPERCYCLIC_ROS_CONVERTER_H_
#define _KORTEX_GRIPPERCYCLIC_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <GripperCyclic.pb.h>

#include "kortex_driver/generated/common_ros_converter.h"
#include "kortex_driver/generated/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/base_ros_converter.h"
#include "kortex_driver/generated/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/basecyclic_ros_converter.h"
#include "kortex_driver/generated/controlconfig_ros_converter.h"
#include "kortex_driver/generated/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/devicemanager_ros_converter.h"
#include "kortex_driver/generated/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/visionconfig_ros_converter.h"


#include "kortex_driver/GripperCyclic_MessageId.h"
#include "kortex_driver/MotorCommand.h"
#include "kortex_driver/GripperCyclic_Command.h"
#include "kortex_driver/MotorFeedback.h"
#include "kortex_driver/GripperCyclic_Feedback.h"
#include "kortex_driver/CustomDataUnit.h"
#include "kortex_driver/GripperCyclic_CustomData.h"


int ToRosData(Kinova::Api::GripperCyclic::MessageId input, kortex_driver::GripperCyclic_MessageId &output);
int ToRosData(Kinova::Api::GripperCyclic::MotorCommand input, kortex_driver::MotorCommand &output);
int ToRosData(Kinova::Api::GripperCyclic::Command input, kortex_driver::GripperCyclic_Command &output);
int ToRosData(Kinova::Api::GripperCyclic::MotorFeedback input, kortex_driver::MotorFeedback &output);
int ToRosData(Kinova::Api::GripperCyclic::Feedback input, kortex_driver::GripperCyclic_Feedback &output);
int ToRosData(Kinova::Api::GripperCyclic::CustomDataUnit input, kortex_driver::CustomDataUnit &output);
int ToRosData(Kinova::Api::GripperCyclic::CustomData input, kortex_driver::GripperCyclic_CustomData &output);

#endif