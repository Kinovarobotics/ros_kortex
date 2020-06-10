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
 
#ifndef _KORTEX_BASECYCLIC_PROTO_CONVERTER_H_
#define _KORTEX_BASECYCLIC_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <BaseCyclic.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


#include "kortex_driver/ActuatorCommand.h"
#include "kortex_driver/ActuatorFeedback.h"
#include "kortex_driver/ActuatorCustomData.h"
#include "kortex_driver/BaseFeedback.h"
#include "kortex_driver/BaseCyclic_CustomData.h"
#include "kortex_driver/BaseCyclic_Command.h"
#include "kortex_driver/BaseCyclic_Feedback.h"


int ToProtoData(kortex_driver::ActuatorCommand input, Kinova::Api::BaseCyclic::ActuatorCommand *output);
int ToProtoData(kortex_driver::ActuatorFeedback input, Kinova::Api::BaseCyclic::ActuatorFeedback *output);
int ToProtoData(kortex_driver::ActuatorCustomData input, Kinova::Api::BaseCyclic::ActuatorCustomData *output);
int ToProtoData(kortex_driver::BaseFeedback input, Kinova::Api::BaseCyclic::BaseFeedback *output);
int ToProtoData(kortex_driver::BaseCyclic_CustomData input, Kinova::Api::BaseCyclic::CustomData *output);
int ToProtoData(kortex_driver::BaseCyclic_Command input, Kinova::Api::BaseCyclic::Command *output);
int ToProtoData(kortex_driver::BaseCyclic_Feedback input, Kinova::Api::BaseCyclic::Feedback *output);

#endif