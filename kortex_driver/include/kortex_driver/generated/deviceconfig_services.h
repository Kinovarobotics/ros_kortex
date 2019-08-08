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
 
#ifndef _KORTEX_DEVICECONFIG_SERVICES_H_
#define _KORTEX_DEVICECONFIG_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <DeviceConfig.pb.h>
#include <DeviceConfigClientRpc.h>
#include "kortex_driver/GetRunMode.h"
#include "kortex_driver/SetRunMode.h"
#include "kortex_driver/GetDeviceType.h"
#include "kortex_driver/GetFirmwareVersion.h"
#include "kortex_driver/GetBootloaderVersion.h"
#include "kortex_driver/GetModelNumber.h"
#include "kortex_driver/GetPartNumber.h"
#include "kortex_driver/GetSerialNumber.h"
#include "kortex_driver/GetMACAddress.h"
#include "kortex_driver/GetIPv4Settings.h"
#include "kortex_driver/SetIPv4Settings.h"
#include "kortex_driver/GetPartNumberRevision.h"
#include "kortex_driver/RebootRequest.h"
#include "kortex_driver/SetSafetyEnable.h"
#include "kortex_driver/SetSafetyErrorThreshold.h"
#include "kortex_driver/SetSafetyWarningThreshold.h"
#include "kortex_driver/SetSafetyConfiguration.h"
#include "kortex_driver/GetSafetyConfiguration.h"
#include "kortex_driver/GetSafetyInformation.h"
#include "kortex_driver/GetSafetyEnable.h"
#include "kortex_driver/GetSafetyStatus.h"
#include "kortex_driver/ClearAllSafetyStatus.h"
#include "kortex_driver/ClearSafetyStatus.h"
#include "kortex_driver/GetAllSafetyConfiguration.h"
#include "kortex_driver/GetAllSafetyInformation.h"
#include "kortex_driver/ResetSafetyDefaults.h"
#include "kortex_driver/OnNotificationSafetyTopic.h"
#include "kortex_driver/SafetyNotification.h"
#include "kortex_driver/ExecuteCalibration.h"
#include "kortex_driver/GetCalibrationResult.h"
#include "kortex_driver/StopCalibration.h"
#include "kortex_driver/DeviceConfig_SetCapSenseConfig.h"
#include "kortex_driver/DeviceConfig_GetCapSenseConfig.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class DeviceConfigServices
{
    public:
        DeviceConfigServices(ros::NodeHandle& n, Kinova::Api::DeviceConfig::DeviceConfigClient* deviceconfig, uint32_t device_id, uint32_t timeout_ms);

        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);
        bool GetRunMode(kortex_driver::GetRunMode::Request  &req, kortex_driver::GetRunMode::Response &res);
        bool SetRunMode(kortex_driver::SetRunMode::Request  &req, kortex_driver::SetRunMode::Response &res);
        bool GetDeviceType(kortex_driver::GetDeviceType::Request  &req, kortex_driver::GetDeviceType::Response &res);
        bool GetFirmwareVersion(kortex_driver::GetFirmwareVersion::Request  &req, kortex_driver::GetFirmwareVersion::Response &res);
        bool GetBootloaderVersion(kortex_driver::GetBootloaderVersion::Request  &req, kortex_driver::GetBootloaderVersion::Response &res);
        bool GetModelNumber(kortex_driver::GetModelNumber::Request  &req, kortex_driver::GetModelNumber::Response &res);
        bool GetPartNumber(kortex_driver::GetPartNumber::Request  &req, kortex_driver::GetPartNumber::Response &res);
        bool GetSerialNumber(kortex_driver::GetSerialNumber::Request  &req, kortex_driver::GetSerialNumber::Response &res);
        bool GetMACAddress(kortex_driver::GetMACAddress::Request  &req, kortex_driver::GetMACAddress::Response &res);
        bool GetIPv4Settings(kortex_driver::GetIPv4Settings::Request  &req, kortex_driver::GetIPv4Settings::Response &res);
        bool SetIPv4Settings(kortex_driver::SetIPv4Settings::Request  &req, kortex_driver::SetIPv4Settings::Response &res);
        bool GetPartNumberRevision(kortex_driver::GetPartNumberRevision::Request  &req, kortex_driver::GetPartNumberRevision::Response &res);
        bool RebootRequest(kortex_driver::RebootRequest::Request  &req, kortex_driver::RebootRequest::Response &res);
        bool SetSafetyEnable(kortex_driver::SetSafetyEnable::Request  &req, kortex_driver::SetSafetyEnable::Response &res);
        bool SetSafetyErrorThreshold(kortex_driver::SetSafetyErrorThreshold::Request  &req, kortex_driver::SetSafetyErrorThreshold::Response &res);
        bool SetSafetyWarningThreshold(kortex_driver::SetSafetyWarningThreshold::Request  &req, kortex_driver::SetSafetyWarningThreshold::Response &res);
        bool SetSafetyConfiguration(kortex_driver::SetSafetyConfiguration::Request  &req, kortex_driver::SetSafetyConfiguration::Response &res);
        bool GetSafetyConfiguration(kortex_driver::GetSafetyConfiguration::Request  &req, kortex_driver::GetSafetyConfiguration::Response &res);
        bool GetSafetyInformation(kortex_driver::GetSafetyInformation::Request  &req, kortex_driver::GetSafetyInformation::Response &res);
        bool GetSafetyEnable(kortex_driver::GetSafetyEnable::Request  &req, kortex_driver::GetSafetyEnable::Response &res);
        bool GetSafetyStatus(kortex_driver::GetSafetyStatus::Request  &req, kortex_driver::GetSafetyStatus::Response &res);
        bool ClearAllSafetyStatus(kortex_driver::ClearAllSafetyStatus::Request  &req, kortex_driver::ClearAllSafetyStatus::Response &res);
        bool ClearSafetyStatus(kortex_driver::ClearSafetyStatus::Request  &req, kortex_driver::ClearSafetyStatus::Response &res);
        bool GetAllSafetyConfiguration(kortex_driver::GetAllSafetyConfiguration::Request  &req, kortex_driver::GetAllSafetyConfiguration::Response &res);
        bool GetAllSafetyInformation(kortex_driver::GetAllSafetyInformation::Request  &req, kortex_driver::GetAllSafetyInformation::Response &res);
        bool ResetSafetyDefaults(kortex_driver::ResetSafetyDefaults::Request  &req, kortex_driver::ResetSafetyDefaults::Response &res);
        bool OnNotificationSafetyTopic(kortex_driver::OnNotificationSafetyTopic::Request  &req, kortex_driver::OnNotificationSafetyTopic::Response &res);
        void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif);
        bool ExecuteCalibration(kortex_driver::ExecuteCalibration::Request  &req, kortex_driver::ExecuteCalibration::Response &res);
        bool GetCalibrationResult(kortex_driver::GetCalibrationResult::Request  &req, kortex_driver::GetCalibrationResult::Response &res);
        bool StopCalibration(kortex_driver::StopCalibration::Request  &req, kortex_driver::StopCalibration::Response &res);
        bool DeviceConfig_SetCapSenseConfig(kortex_driver::DeviceConfig_SetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_SetCapSenseConfig::Response &res);
        bool DeviceConfig_GetCapSenseConfig(kortex_driver::DeviceConfig_GetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_GetCapSenseConfig::Response &res);

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::DeviceConfig::DeviceConfigClient* m_deviceconfig;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_SafetyTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceGetRunMode;
	ros::ServiceServer m_serviceSetRunMode;
	ros::ServiceServer m_serviceGetDeviceType;
	ros::ServiceServer m_serviceGetFirmwareVersion;
	ros::ServiceServer m_serviceGetBootloaderVersion;
	ros::ServiceServer m_serviceGetModelNumber;
	ros::ServiceServer m_serviceGetPartNumber;
	ros::ServiceServer m_serviceGetSerialNumber;
	ros::ServiceServer m_serviceGetMACAddress;
	ros::ServiceServer m_serviceGetIPv4Settings;
	ros::ServiceServer m_serviceSetIPv4Settings;
	ros::ServiceServer m_serviceGetPartNumberRevision;
	ros::ServiceServer m_serviceRebootRequest;
	ros::ServiceServer m_serviceSetSafetyEnable;
	ros::ServiceServer m_serviceSetSafetyErrorThreshold;
	ros::ServiceServer m_serviceSetSafetyWarningThreshold;
	ros::ServiceServer m_serviceSetSafetyConfiguration;
	ros::ServiceServer m_serviceGetSafetyConfiguration;
	ros::ServiceServer m_serviceGetSafetyInformation;
	ros::ServiceServer m_serviceGetSafetyEnable;
	ros::ServiceServer m_serviceGetSafetyStatus;
	ros::ServiceServer m_serviceClearAllSafetyStatus;
	ros::ServiceServer m_serviceClearSafetyStatus;
	ros::ServiceServer m_serviceGetAllSafetyConfiguration;
	ros::ServiceServer m_serviceGetAllSafetyInformation;
	ros::ServiceServer m_serviceResetSafetyDefaults;
	ros::ServiceServer m_serviceOnNotificationSafetyTopic;
	ros::ServiceServer m_serviceExecuteCalibration;
	ros::ServiceServer m_serviceGetCalibrationResult;
	ros::ServiceServer m_serviceStopCalibration;
	ros::ServiceServer m_serviceDeviceConfig_SetCapSenseConfig;
	ros::ServiceServer m_serviceDeviceConfig_GetCapSenseConfig;
};
#endif
