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
 
#ifndef _KORTEX_INTERCONNECTCYCLIC_PROTO_CONVERTER_H_
#define _KORTEX_INTERCONNECTCYCLIC_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <InterconnectCyclic.pb.h>

#include "kortex_driver/generated/common_proto_converter.h"
#include "kortex_driver/generated/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/base_proto_converter.h"
#include "kortex_driver/generated/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/basecyclic_proto_converter.h"
#include "kortex_driver/generated/controlconfig_proto_converter.h"
#include "kortex_driver/generated/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/devicemanager_proto_converter.h"
#include "kortex_driver/generated/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/visionconfig_proto_converter.h"


#include "kortex_driver/InterconnectCyclic_MessageId.h"
#include "kortex_driver/InterconnectCyclic_Command.h"
#include "kortex_driver/InterconnectCyclic_Feedback.h"
#include "kortex_driver/InterconnectCyclic_CustomData.h"


int ToProtoData(kortex_driver::InterconnectCyclic_MessageId input, Kinova::Api::InterconnectCyclic::MessageId *output);
int ToProtoData(kortex_driver::InterconnectCyclic_Command input, Kinova::Api::InterconnectCyclic::Command *output);
int ToProtoData(kortex_driver::InterconnectCyclic_Feedback input, Kinova::Api::InterconnectCyclic::Feedback *output);
int ToProtoData(kortex_driver::InterconnectCyclic_CustomData input, Kinova::Api::InterconnectCyclic::CustomData *output);

#endif