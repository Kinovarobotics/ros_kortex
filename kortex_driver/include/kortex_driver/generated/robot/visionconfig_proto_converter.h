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
 
#ifndef _KORTEX_VISIONCONFIG_PROTO_CONVERTER_H_
#define _KORTEX_VISIONCONFIG_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <VisionConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"


#include "kortex_driver/SensorSettings.h"
#include "kortex_driver/SensorIdentifier.h"
#include "kortex_driver/IntrinsicProfileIdentifier.h"
#include "kortex_driver/OptionIdentifier.h"
#include "kortex_driver/OptionValue.h"
#include "kortex_driver/OptionInformation.h"
#include "kortex_driver/SensorFocusAction.h"
#include "kortex_driver/FocusPoint.h"
#include "kortex_driver/ManualFocus.h"
#include "kortex_driver/VisionNotification.h"
#include "kortex_driver/IntrinsicParameters.h"
#include "kortex_driver/DistortionCoefficients.h"
#include "kortex_driver/ExtrinsicParameters.h"
#include "kortex_driver/VisionConfig_RotationMatrix.h"
#include "kortex_driver/VisionConfig_RotationMatrixRow.h"
#include "kortex_driver/TranslationVector.h"


int ToProtoData(kortex_driver::SensorSettings input, Kinova::Api::VisionConfig::SensorSettings *output);
int ToProtoData(kortex_driver::SensorIdentifier input, Kinova::Api::VisionConfig::SensorIdentifier *output);
int ToProtoData(kortex_driver::IntrinsicProfileIdentifier input, Kinova::Api::VisionConfig::IntrinsicProfileIdentifier *output);
int ToProtoData(kortex_driver::OptionIdentifier input, Kinova::Api::VisionConfig::OptionIdentifier *output);
int ToProtoData(kortex_driver::OptionValue input, Kinova::Api::VisionConfig::OptionValue *output);
int ToProtoData(kortex_driver::OptionInformation input, Kinova::Api::VisionConfig::OptionInformation *output);
int ToProtoData(kortex_driver::SensorFocusAction input, Kinova::Api::VisionConfig::SensorFocusAction *output);
int ToProtoData(kortex_driver::FocusPoint input, Kinova::Api::VisionConfig::FocusPoint *output);
int ToProtoData(kortex_driver::ManualFocus input, Kinova::Api::VisionConfig::ManualFocus *output);
int ToProtoData(kortex_driver::VisionNotification input, Kinova::Api::VisionConfig::VisionNotification *output);
int ToProtoData(kortex_driver::IntrinsicParameters input, Kinova::Api::VisionConfig::IntrinsicParameters *output);
int ToProtoData(kortex_driver::DistortionCoefficients input, Kinova::Api::VisionConfig::DistortionCoefficients *output);
int ToProtoData(kortex_driver::ExtrinsicParameters input, Kinova::Api::VisionConfig::ExtrinsicParameters *output);
int ToProtoData(kortex_driver::VisionConfig_RotationMatrix input, Kinova::Api::VisionConfig::RotationMatrix *output);
int ToProtoData(kortex_driver::VisionConfig_RotationMatrixRow input, Kinova::Api::VisionConfig::RotationMatrixRow *output);
int ToProtoData(kortex_driver::TranslationVector input, Kinova::Api::VisionConfig::TranslationVector *output);

#endif