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
 
#ifndef _KORTEX_DeviceConfigPROTO_CONVERTER_H_
#define _KORTEX_DeviceConfigPROTO_CONVERTER_H_

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

int ToProtoData(kortex_device_manager::DeviceType intput, DeviceType *output);
int ToProtoData(kortex_device_manager::RunMode intput, RunMode *output);
int ToProtoData(kortex_device_manager::FirmwareVersion intput, FirmwareVersion *output);
int ToProtoData(kortex_device_manager::BootloaderVersion intput, BootloaderVersion *output);
int ToProtoData(kortex_device_manager::ModelNumber intput, ModelNumber *output);
int ToProtoData(kortex_device_manager::PartNumber intput, PartNumber *output);
int ToProtoData(kortex_device_manager::SerialNumber intput, SerialNumber *output);
int ToProtoData(kortex_device_manager::MACAddress intput, MACAddress *output);
int ToProtoData(kortex_device_manager::IPv4Settings intput, IPv4Settings *output);
int ToProtoData(kortex_device_manager::PartNumberRevision intput, PartNumberRevision *output);
int ToProtoData(kortex_device_manager::PowerOnSelfTestResult intput, PowerOnSelfTestResult *output);
int ToProtoData(kortex_device_manager::RebootRqst intput, RebootRqst *output);
int ToProtoData(kortex_device_manager::SafetyInformation intput, SafetyInformation *output);
int ToProtoData(kortex_device_manager::SafetyInformationList intput, SafetyInformationList *output);
int ToProtoData(kortex_device_manager::SafetyEnable intput, SafetyEnable *output);
int ToProtoData(kortex_device_manager::SafetyThreshold intput, SafetyThreshold *output);
int ToProtoData(kortex_device_manager::SafetyConfiguration intput, SafetyConfiguration *output);
int ToProtoData(kortex_device_manager::SafetyConfigurationList intput, SafetyConfigurationList *output);
int ToProtoData(kortex_device_manager::SafetyStatus intput, SafetyStatus *output);

#endif