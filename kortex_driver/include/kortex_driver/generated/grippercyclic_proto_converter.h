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
 
#ifndef _KORTEX_GRIPPERCYCLIC_PROTO_CONVERTER_H_
#define _KORTEX_GRIPPERCYCLIC_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <GripperCyclic.pb.h>

#include "kortex_driver/generated/common_proto_converter.h"
#include "kortex_driver/generated/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/base_proto_converter.h"
#include "kortex_driver/generated/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/basecyclic_proto_converter.h"
#include "kortex_driver/generated/controlconfig_proto_converter.h"
#include "kortex_driver/generated/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/devicemanager_proto_converter.h"
#include "kortex_driver/generated/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/visionconfig_proto_converter.h"


#include "kortex_driver/GripperCyclic_MessageId.h"
#include "kortex_driver/MotorCommand.h"
#include "kortex_driver/GripperCyclic_Command.h"
#include "kortex_driver/MotorFeedback.h"
#include "kortex_driver/GripperCyclic_Feedback.h"
#include "kortex_driver/CustomDataUnit.h"
#include "kortex_driver/GripperCyclic_CustomData.h"


int ToProtoData(kortex_driver::GripperCyclic_MessageId input, Kinova::Api::GripperCyclic::MessageId *output);
int ToProtoData(kortex_driver::MotorCommand input, Kinova::Api::GripperCyclic::MotorCommand *output);
int ToProtoData(kortex_driver::GripperCyclic_Command input, Kinova::Api::GripperCyclic::Command *output);
int ToProtoData(kortex_driver::MotorFeedback input, Kinova::Api::GripperCyclic::MotorFeedback *output);
int ToProtoData(kortex_driver::GripperCyclic_Feedback input, Kinova::Api::GripperCyclic::Feedback *output);
int ToProtoData(kortex_driver::CustomDataUnit input, Kinova::Api::GripperCyclic::CustomDataUnit *output);
int ToProtoData(kortex_driver::GripperCyclic_CustomData input, Kinova::Api::GripperCyclic::CustomData *output);

#endif