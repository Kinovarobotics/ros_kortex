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
 
#ifndef _KORTEX_DEVICECONFIG_PROTO_CONVERTER_H_
#define _KORTEX_DEVICECONFIG_PROTO_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <DeviceConfig.pb.h>

#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"


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


int ToProtoData(kortex_driver::DeviceType input, Kinova::Api::DeviceConfig::DeviceType *output);
int ToProtoData(kortex_driver::RunMode input, Kinova::Api::DeviceConfig::RunMode *output);
int ToProtoData(kortex_driver::FirmwareVersion input, Kinova::Api::DeviceConfig::FirmwareVersion *output);
int ToProtoData(kortex_driver::BootloaderVersion input, Kinova::Api::DeviceConfig::BootloaderVersion *output);
int ToProtoData(kortex_driver::ModelNumber input, Kinova::Api::DeviceConfig::ModelNumber *output);
int ToProtoData(kortex_driver::PartNumber input, Kinova::Api::DeviceConfig::PartNumber *output);
int ToProtoData(kortex_driver::SerialNumber input, Kinova::Api::DeviceConfig::SerialNumber *output);
int ToProtoData(kortex_driver::MACAddress input, Kinova::Api::DeviceConfig::MACAddress *output);
int ToProtoData(kortex_driver::IPv4Settings input, Kinova::Api::DeviceConfig::IPv4Settings *output);
int ToProtoData(kortex_driver::PartNumberRevision input, Kinova::Api::DeviceConfig::PartNumberRevision *output);
int ToProtoData(kortex_driver::PowerOnSelfTestResult input, Kinova::Api::DeviceConfig::PowerOnSelfTestResult *output);
int ToProtoData(kortex_driver::RebootRqst input, Kinova::Api::DeviceConfig::RebootRqst *output);
int ToProtoData(kortex_driver::SafetyInformation input, Kinova::Api::DeviceConfig::SafetyInformation *output);
int ToProtoData(kortex_driver::SafetyInformationList input, Kinova::Api::DeviceConfig::SafetyInformationList *output);
int ToProtoData(kortex_driver::SafetyEnable input, Kinova::Api::DeviceConfig::SafetyEnable *output);
int ToProtoData(kortex_driver::SafetyThreshold input, Kinova::Api::DeviceConfig::SafetyThreshold *output);
int ToProtoData(kortex_driver::SafetyConfiguration input, Kinova::Api::DeviceConfig::SafetyConfiguration *output);
int ToProtoData(kortex_driver::SafetyConfigurationList input, Kinova::Api::DeviceConfig::SafetyConfigurationList *output);
int ToProtoData(kortex_driver::SafetyStatus input, Kinova::Api::DeviceConfig::SafetyStatus *output);
int ToProtoData(kortex_driver::CalibrationParameter input, Kinova::Api::DeviceConfig::CalibrationParameter *output);
int ToProtoData(kortex_driver::Calibration input, Kinova::Api::DeviceConfig::Calibration *output);
int ToProtoData(kortex_driver::CalibrationElement input, Kinova::Api::DeviceConfig::CalibrationElement *output);
int ToProtoData(kortex_driver::CalibrationResult input, Kinova::Api::DeviceConfig::CalibrationResult *output);
int ToProtoData(kortex_driver::DeviceConfig_CapSenseConfig input, Kinova::Api::DeviceConfig::CapSenseConfig *output);
int ToProtoData(kortex_driver::CapSenseRegister input, Kinova::Api::DeviceConfig::CapSenseRegister *output);

#endif