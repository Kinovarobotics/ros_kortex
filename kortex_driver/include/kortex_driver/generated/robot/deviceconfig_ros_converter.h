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
 
#ifndef _KORTEX_DEVICECONFIG_ROS_CONVERTER_H_
#define _KORTEX_DEVICECONFIG_ROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <DeviceConfig.pb.h>

#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"


#include "kortex_driver/DeviceType.h"
#include "kortex_driver/RunMode.h"
#include "kortex_driver/FirmwareVersion.h"
#include "kortex_driver/BootloaderVersion.h"
#include "kortex_driver/ModelNumber.h"
#include "kortex_driver/PartNumber.h"
#include "kortex_driver/SerialNumber.h"
#include "kortex_driver/MACAddress.h"
#include "kortex_driver/IPv4Settings.h"
#include "kortex_driver/PartNumberRevision.h"
#include "kortex_driver/PowerOnSelfTestResult.h"
#include "kortex_driver/RebootRqst.h"
#include "kortex_driver/SafetyInformation.h"
#include "kortex_driver/SafetyInformationList.h"
#include "kortex_driver/SafetyEnable.h"
#include "kortex_driver/SafetyThreshold.h"
#include "kortex_driver/SafetyConfiguration.h"
#include "kortex_driver/SafetyConfigurationList.h"
#include "kortex_driver/SafetyStatus.h"
#include "kortex_driver/CalibrationParameter.h"
#include "kortex_driver/Calibration.h"
#include "kortex_driver/CalibrationElement.h"
#include "kortex_driver/CalibrationResult.h"
#include "kortex_driver/DeviceConfig_CapSenseConfig.h"
#include "kortex_driver/CapSenseRegister.h"


int ToRosData(Kinova::Api::DeviceConfig::DeviceType input, kortex_driver::DeviceType &output);
int ToRosData(Kinova::Api::DeviceConfig::RunMode input, kortex_driver::RunMode &output);
int ToRosData(Kinova::Api::DeviceConfig::FirmwareVersion input, kortex_driver::FirmwareVersion &output);
int ToRosData(Kinova::Api::DeviceConfig::BootloaderVersion input, kortex_driver::BootloaderVersion &output);
int ToRosData(Kinova::Api::DeviceConfig::ModelNumber input, kortex_driver::ModelNumber &output);
int ToRosData(Kinova::Api::DeviceConfig::PartNumber input, kortex_driver::PartNumber &output);
int ToRosData(Kinova::Api::DeviceConfig::SerialNumber input, kortex_driver::SerialNumber &output);
int ToRosData(Kinova::Api::DeviceConfig::MACAddress input, kortex_driver::MACAddress &output);
int ToRosData(Kinova::Api::DeviceConfig::IPv4Settings input, kortex_driver::IPv4Settings &output);
int ToRosData(Kinova::Api::DeviceConfig::PartNumberRevision input, kortex_driver::PartNumberRevision &output);
int ToRosData(Kinova::Api::DeviceConfig::PowerOnSelfTestResult input, kortex_driver::PowerOnSelfTestResult &output);
int ToRosData(Kinova::Api::DeviceConfig::RebootRqst input, kortex_driver::RebootRqst &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyInformation input, kortex_driver::SafetyInformation &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyInformationList input, kortex_driver::SafetyInformationList &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyEnable input, kortex_driver::SafetyEnable &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyThreshold input, kortex_driver::SafetyThreshold &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyConfiguration input, kortex_driver::SafetyConfiguration &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyConfigurationList input, kortex_driver::SafetyConfigurationList &output);
int ToRosData(Kinova::Api::DeviceConfig::SafetyStatus input, kortex_driver::SafetyStatus &output);
int ToRosData(Kinova::Api::DeviceConfig::CalibrationParameter input, kortex_driver::CalibrationParameter &output);
int ToRosData(Kinova::Api::DeviceConfig::Calibration input, kortex_driver::Calibration &output);
int ToRosData(Kinova::Api::DeviceConfig::CalibrationElement input, kortex_driver::CalibrationElement &output);
int ToRosData(Kinova::Api::DeviceConfig::CalibrationResult input, kortex_driver::CalibrationResult &output);
int ToRosData(Kinova::Api::DeviceConfig::CapSenseConfig input, kortex_driver::DeviceConfig_CapSenseConfig &output);
int ToRosData(Kinova::Api::DeviceConfig::CapSenseRegister input, kortex_driver::CapSenseRegister &output);

#endif