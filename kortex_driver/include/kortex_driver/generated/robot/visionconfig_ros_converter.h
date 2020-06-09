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
 
#ifndef _KORTEX_VISIONCONFIG_ROS_CONVERTER_H_
#define _KORTEX_VISIONCONFIG_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <VisionConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"


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


int ToRosData(Kinova::Api::VisionConfig::SensorSettings input, kortex_driver::SensorSettings &output);
int ToRosData(Kinova::Api::VisionConfig::SensorIdentifier input, kortex_driver::SensorIdentifier &output);
int ToRosData(Kinova::Api::VisionConfig::IntrinsicProfileIdentifier input, kortex_driver::IntrinsicProfileIdentifier &output);
int ToRosData(Kinova::Api::VisionConfig::OptionIdentifier input, kortex_driver::OptionIdentifier &output);
int ToRosData(Kinova::Api::VisionConfig::OptionValue input, kortex_driver::OptionValue &output);
int ToRosData(Kinova::Api::VisionConfig::OptionInformation input, kortex_driver::OptionInformation &output);
int ToRosData(Kinova::Api::VisionConfig::SensorFocusAction input, kortex_driver::SensorFocusAction &output);
int ToRosData(Kinova::Api::VisionConfig::FocusPoint input, kortex_driver::FocusPoint &output);
int ToRosData(Kinova::Api::VisionConfig::ManualFocus input, kortex_driver::ManualFocus &output);
int ToRosData(Kinova::Api::VisionConfig::VisionNotification input, kortex_driver::VisionNotification &output);
int ToRosData(Kinova::Api::VisionConfig::IntrinsicParameters input, kortex_driver::IntrinsicParameters &output);
int ToRosData(Kinova::Api::VisionConfig::DistortionCoefficients input, kortex_driver::DistortionCoefficients &output);
int ToRosData(Kinova::Api::VisionConfig::ExtrinsicParameters input, kortex_driver::ExtrinsicParameters &output);
int ToRosData(Kinova::Api::VisionConfig::RotationMatrix input, kortex_driver::VisionConfig_RotationMatrix &output);
int ToRosData(Kinova::Api::VisionConfig::RotationMatrixRow input, kortex_driver::VisionConfig_RotationMatrixRow &output);
int ToRosData(Kinova::Api::VisionConfig::TranslationVector input, kortex_driver::TranslationVector &output);

#endif