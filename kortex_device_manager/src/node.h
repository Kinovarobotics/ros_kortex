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
 
#ifndef _KORTEX_SERVICES_H_
#define _KORTEX_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <Frame.pb.h>
#include <Session.pb.h>
#include <Common.pb.h>
#include <DeviceConfig.pb.h>
#include <DeviceManager.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include "kortex_device_manager/GetRunMode.h"
#include "kortex_device_manager/SetRunMode.h"
#include "kortex_device_manager/GetDeviceType.h"
#include "kortex_device_manager/GetFirmwareVersion.h"
#include "kortex_device_manager/GetBootloaderVersion.h"
#include "kortex_device_manager/GetModelNumber.h"
#include "kortex_device_manager/GetPartNumber.h"
#include "kortex_device_manager/GetSerialNumber.h"
#include "kortex_device_manager/GetMACAddress.h"
#include "kortex_device_manager/GetIPv4Settings.h"
#include "kortex_device_manager/SetIPv4Settings.h"
#include "kortex_device_manager/GetPartNumberRevision.h"
#include "kortex_device_manager/GetPowerOnSelfTestResult.h"
#include "kortex_device_manager/RebootRequest.h"
#include "kortex_device_manager/SetSafetyEnable.h"
#include "kortex_device_manager/SetSafetyErrorThreshold.h"
#include "kortex_device_manager/SetSafetyWarningThreshold.h"
#include "kortex_device_manager/SetSafetyConfiguration.h"
#include "kortex_device_manager/GetSafetyConfiguration.h"
#include "kortex_device_manager/GetSafetyInformation.h"
#include "kortex_device_manager/GetSafetyEnable.h"
#include "kortex_device_manager/GetSafetyStatus.h"
#include "kortex_device_manager/ClearAllSafetyStatus.h"
#include "kortex_device_manager/ClearSafetyStatus.h"
#include "kortex_device_manager/GetAllSafetyConfiguration.h"
#include "kortex_device_manager/GetAllSafetyInformation.h"
#include "kortex_device_manager/ResetSafetyDefaults.h"
#include "kortex_device_manager/OnNotificationSafetyTopic.h"
#include "kortex_device_manager/SafetyNotification.h"
#include "kortex_device_manager/SetModelNumber.h"
#include "kortex_device_manager/SetPartNumber.h"
#include "kortex_device_manager/SetPartNumberRevision.h"
#include "kortex_device_manager/SetSerialNumber.h"
#include "kortex_device_manager/SetMACAddress.h"
#include "kortex_device_manager/ReadAllDevices.h"
#include "kortex_device_manager/KortexError.h"

using namespace std;
using namespace Kinova::Api;
using namespace Kinova::Api::Common;
using namespace Kinova::Api::DeviceConfig;
using namespace Kinova::Api::DeviceManager;

class KortexDeviceManager
{
    public:
        KortexDeviceManager(char* ip, ros::NodeHandle& n);


        bool GetRunMode(kortex_device_manager::GetRunMode::Request  &req, kortex_device_manager::GetRunMode::Response &res);
        bool SetRunMode(kortex_device_manager::SetRunMode::Request  &req, kortex_device_manager::SetRunMode::Response &res);
        bool GetDeviceType(kortex_device_manager::GetDeviceType::Request  &req, kortex_device_manager::GetDeviceType::Response &res);
        bool GetFirmwareVersion(kortex_device_manager::GetFirmwareVersion::Request  &req, kortex_device_manager::GetFirmwareVersion::Response &res);
        bool GetBootloaderVersion(kortex_device_manager::GetBootloaderVersion::Request  &req, kortex_device_manager::GetBootloaderVersion::Response &res);
        bool GetModelNumber(kortex_device_manager::GetModelNumber::Request  &req, kortex_device_manager::GetModelNumber::Response &res);
        bool GetPartNumber(kortex_device_manager::GetPartNumber::Request  &req, kortex_device_manager::GetPartNumber::Response &res);
        bool GetSerialNumber(kortex_device_manager::GetSerialNumber::Request  &req, kortex_device_manager::GetSerialNumber::Response &res);
        bool GetMACAddress(kortex_device_manager::GetMACAddress::Request  &req, kortex_device_manager::GetMACAddress::Response &res);
        bool GetIPv4Settings(kortex_device_manager::GetIPv4Settings::Request  &req, kortex_device_manager::GetIPv4Settings::Response &res);
        bool SetIPv4Settings(kortex_device_manager::SetIPv4Settings::Request  &req, kortex_device_manager::SetIPv4Settings::Response &res);
        bool GetPartNumberRevision(kortex_device_manager::GetPartNumberRevision::Request  &req, kortex_device_manager::GetPartNumberRevision::Response &res);
        bool GetPowerOnSelfTestResult(kortex_device_manager::GetPowerOnSelfTestResult::Request  &req, kortex_device_manager::GetPowerOnSelfTestResult::Response &res);
        bool RebootRequest(kortex_device_manager::RebootRequest::Request  &req, kortex_device_manager::RebootRequest::Response &res);
        bool SetSafetyEnable(kortex_device_manager::SetSafetyEnable::Request  &req, kortex_device_manager::SetSafetyEnable::Response &res);
        bool SetSafetyErrorThreshold(kortex_device_manager::SetSafetyErrorThreshold::Request  &req, kortex_device_manager::SetSafetyErrorThreshold::Response &res);
        bool SetSafetyWarningThreshold(kortex_device_manager::SetSafetyWarningThreshold::Request  &req, kortex_device_manager::SetSafetyWarningThreshold::Response &res);
        bool SetSafetyConfiguration(kortex_device_manager::SetSafetyConfiguration::Request  &req, kortex_device_manager::SetSafetyConfiguration::Response &res);
        bool GetSafetyConfiguration(kortex_device_manager::GetSafetyConfiguration::Request  &req, kortex_device_manager::GetSafetyConfiguration::Response &res);
        bool GetSafetyInformation(kortex_device_manager::GetSafetyInformation::Request  &req, kortex_device_manager::GetSafetyInformation::Response &res);
        bool GetSafetyEnable(kortex_device_manager::GetSafetyEnable::Request  &req, kortex_device_manager::GetSafetyEnable::Response &res);
        bool GetSafetyStatus(kortex_device_manager::GetSafetyStatus::Request  &req, kortex_device_manager::GetSafetyStatus::Response &res);
        bool ClearAllSafetyStatus(kortex_device_manager::ClearAllSafetyStatus::Request  &req, kortex_device_manager::ClearAllSafetyStatus::Response &res);
        bool ClearSafetyStatus(kortex_device_manager::ClearSafetyStatus::Request  &req, kortex_device_manager::ClearSafetyStatus::Response &res);
        bool GetAllSafetyConfiguration(kortex_device_manager::GetAllSafetyConfiguration::Request  &req, kortex_device_manager::GetAllSafetyConfiguration::Response &res);
        bool GetAllSafetyInformation(kortex_device_manager::GetAllSafetyInformation::Request  &req, kortex_device_manager::GetAllSafetyInformation::Response &res);
        bool ResetSafetyDefaults(kortex_device_manager::ResetSafetyDefaults::Request  &req, kortex_device_manager::ResetSafetyDefaults::Response &res);
        bool OnNotificationSafetyTopic(kortex_device_manager::OnNotificationSafetyTopic::Request  &req, kortex_device_manager::OnNotificationSafetyTopic::Response &res);
        void cb_SafetyTopic(SafetyNotification notif);
        bool SetModelNumber(kortex_device_manager::SetModelNumber::Request  &req, kortex_device_manager::SetModelNumber::Response &res);
        bool SetPartNumber(kortex_device_manager::SetPartNumber::Request  &req, kortex_device_manager::SetPartNumber::Response &res);
        bool SetPartNumberRevision(kortex_device_manager::SetPartNumberRevision::Request  &req, kortex_device_manager::SetPartNumberRevision::Response &res);
        bool SetSerialNumber(kortex_device_manager::SetSerialNumber::Request  &req, kortex_device_manager::SetSerialNumber::Response &res);
        bool SetMACAddress(kortex_device_manager::SetMACAddress::Request  &req, kortex_device_manager::SetMACAddress::Response &res);

        bool ReadAllDevices(kortex_device_manager::ReadAllDevices::Request  &req, kortex_device_manager::ReadAllDevices::Response &res);


private:
    	TransportClientUdp* m_transport;
    	RouterClient*       m_router;
        
        DeviceConfigClient*   m_deviceconfig;
        DeviceManagerClient*   m_devicemanager;

        SessionManager* m_SessionManager;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_SafetyTopic;
};
#endif
