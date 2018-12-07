/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
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
 
#ifndef _KORTEX_DeviceConfigROS_CONVERTER_H_
#define _KORTEX_DeviceConfigROS_CONVERTER_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <DeviceConfig.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>

#include <BaseCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include "kortex_device_manager/DeviceType.h"
#include "kortex_device_manager/RunMode.h"
#include "kortex_device_manager/FirmwareVersion.h"
#include "kortex_device_manager/BootloaderVersion.h"
#include "kortex_device_manager/ModelNumber.h"
#include "kortex_device_manager/PartNumber.h"
#include "kortex_device_manager/SerialNumber.h"
#include "kortex_device_manager/MACAddress.h"
#include "kortex_device_manager/IPv4Settings.h"
#include "kortex_device_manager/PartNumberRevision.h"
#include "kortex_device_manager/PowerOnSelfTestResult.h"
#include "kortex_device_manager/RebootRqst.h"
#include "kortex_device_manager/SafetyInformation.h"
#include "kortex_device_manager/SafetyInformationList.h"
#include "kortex_device_manager/SafetyEnable.h"
#include "kortex_device_manager/SafetyThreshold.h"
#include "kortex_device_manager/SafetyConfiguration.h"
#include "kortex_device_manager/SafetyConfigurationList.h"
#include "kortex_device_manager/SafetyStatus.h"


using namespace Kinova::Api::DeviceConfig;

int ToRosData(DeviceType input, kortex_device_manager::DeviceType &output);
int ToRosData(RunMode input, kortex_device_manager::RunMode &output);
int ToRosData(FirmwareVersion input, kortex_device_manager::FirmwareVersion &output);
int ToRosData(BootloaderVersion input, kortex_device_manager::BootloaderVersion &output);
int ToRosData(ModelNumber input, kortex_device_manager::ModelNumber &output);
int ToRosData(PartNumber input, kortex_device_manager::PartNumber &output);
int ToRosData(SerialNumber input, kortex_device_manager::SerialNumber &output);
int ToRosData(MACAddress input, kortex_device_manager::MACAddress &output);
int ToRosData(IPv4Settings input, kortex_device_manager::IPv4Settings &output);
int ToRosData(PartNumberRevision input, kortex_device_manager::PartNumberRevision &output);
int ToRosData(PowerOnSelfTestResult input, kortex_device_manager::PowerOnSelfTestResult &output);
int ToRosData(RebootRqst input, kortex_device_manager::RebootRqst &output);
int ToRosData(SafetyInformation input, kortex_device_manager::SafetyInformation &output);
int ToRosData(SafetyInformationList input, kortex_device_manager::SafetyInformationList &output);
int ToRosData(SafetyEnable input, kortex_device_manager::SafetyEnable &output);
int ToRosData(SafetyThreshold input, kortex_device_manager::SafetyThreshold &output);
int ToRosData(SafetyConfiguration input, kortex_device_manager::SafetyConfiguration &output);
int ToRosData(SafetyConfigurationList input, kortex_device_manager::SafetyConfigurationList &output);
int ToRosData(SafetyStatus input, kortex_device_manager::SafetyStatus &output);

#endif