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
 
#ifndef _KORTEX_DEVICECONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_DEVICECONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/deviceconfig_services_interface.h"

using namespace std;

class DeviceConfigSimulationServices : public IDeviceConfigServices
{
    public:
        DeviceConfigSimulationServices(ros::NodeHandle& node_handle);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        std::function<kortex_driver::GetRunMode::Response(const kortex_driver::GetRunMode::Request&)> GetRunModeHandler = nullptr;
        virtual bool GetRunMode(kortex_driver::GetRunMode::Request  &req, kortex_driver::GetRunMode::Response &res) override;
        std::function<kortex_driver::SetRunMode::Response(const kortex_driver::SetRunMode::Request&)> SetRunModeHandler = nullptr;
        virtual bool SetRunMode(kortex_driver::SetRunMode::Request  &req, kortex_driver::SetRunMode::Response &res) override;
        std::function<kortex_driver::GetDeviceType::Response(const kortex_driver::GetDeviceType::Request&)> GetDeviceTypeHandler = nullptr;
        virtual bool GetDeviceType(kortex_driver::GetDeviceType::Request  &req, kortex_driver::GetDeviceType::Response &res) override;
        std::function<kortex_driver::GetFirmwareVersion::Response(const kortex_driver::GetFirmwareVersion::Request&)> GetFirmwareVersionHandler = nullptr;
        virtual bool GetFirmwareVersion(kortex_driver::GetFirmwareVersion::Request  &req, kortex_driver::GetFirmwareVersion::Response &res) override;
        std::function<kortex_driver::GetBootloaderVersion::Response(const kortex_driver::GetBootloaderVersion::Request&)> GetBootloaderVersionHandler = nullptr;
        virtual bool GetBootloaderVersion(kortex_driver::GetBootloaderVersion::Request  &req, kortex_driver::GetBootloaderVersion::Response &res) override;
        std::function<kortex_driver::GetModelNumber::Response(const kortex_driver::GetModelNumber::Request&)> GetModelNumberHandler = nullptr;
        virtual bool GetModelNumber(kortex_driver::GetModelNumber::Request  &req, kortex_driver::GetModelNumber::Response &res) override;
        std::function<kortex_driver::GetPartNumber::Response(const kortex_driver::GetPartNumber::Request&)> GetPartNumberHandler = nullptr;
        virtual bool GetPartNumber(kortex_driver::GetPartNumber::Request  &req, kortex_driver::GetPartNumber::Response &res) override;
        std::function<kortex_driver::GetSerialNumber::Response(const kortex_driver::GetSerialNumber::Request&)> GetSerialNumberHandler = nullptr;
        virtual bool GetSerialNumber(kortex_driver::GetSerialNumber::Request  &req, kortex_driver::GetSerialNumber::Response &res) override;
        std::function<kortex_driver::GetMACAddress::Response(const kortex_driver::GetMACAddress::Request&)> GetMACAddressHandler = nullptr;
        virtual bool GetMACAddress(kortex_driver::GetMACAddress::Request  &req, kortex_driver::GetMACAddress::Response &res) override;
        std::function<kortex_driver::GetIPv4Settings::Response(const kortex_driver::GetIPv4Settings::Request&)> GetIPv4SettingsHandler = nullptr;
        virtual bool GetIPv4Settings(kortex_driver::GetIPv4Settings::Request  &req, kortex_driver::GetIPv4Settings::Response &res) override;
        std::function<kortex_driver::SetIPv4Settings::Response(const kortex_driver::SetIPv4Settings::Request&)> SetIPv4SettingsHandler = nullptr;
        virtual bool SetIPv4Settings(kortex_driver::SetIPv4Settings::Request  &req, kortex_driver::SetIPv4Settings::Response &res) override;
        std::function<kortex_driver::GetPartNumberRevision::Response(const kortex_driver::GetPartNumberRevision::Request&)> GetPartNumberRevisionHandler = nullptr;
        virtual bool GetPartNumberRevision(kortex_driver::GetPartNumberRevision::Request  &req, kortex_driver::GetPartNumberRevision::Response &res) override;
        std::function<kortex_driver::RebootRequest::Response(const kortex_driver::RebootRequest::Request&)> RebootRequestHandler = nullptr;
        virtual bool RebootRequest(kortex_driver::RebootRequest::Request  &req, kortex_driver::RebootRequest::Response &res) override;
        std::function<kortex_driver::SetSafetyEnable::Response(const kortex_driver::SetSafetyEnable::Request&)> SetSafetyEnableHandler = nullptr;
        virtual bool SetSafetyEnable(kortex_driver::SetSafetyEnable::Request  &req, kortex_driver::SetSafetyEnable::Response &res) override;
        std::function<kortex_driver::SetSafetyErrorThreshold::Response(const kortex_driver::SetSafetyErrorThreshold::Request&)> SetSafetyErrorThresholdHandler = nullptr;
        virtual bool SetSafetyErrorThreshold(kortex_driver::SetSafetyErrorThreshold::Request  &req, kortex_driver::SetSafetyErrorThreshold::Response &res) override;
        std::function<kortex_driver::SetSafetyWarningThreshold::Response(const kortex_driver::SetSafetyWarningThreshold::Request&)> SetSafetyWarningThresholdHandler = nullptr;
        virtual bool SetSafetyWarningThreshold(kortex_driver::SetSafetyWarningThreshold::Request  &req, kortex_driver::SetSafetyWarningThreshold::Response &res) override;
        std::function<kortex_driver::SetSafetyConfiguration::Response(const kortex_driver::SetSafetyConfiguration::Request&)> SetSafetyConfigurationHandler = nullptr;
        virtual bool SetSafetyConfiguration(kortex_driver::SetSafetyConfiguration::Request  &req, kortex_driver::SetSafetyConfiguration::Response &res) override;
        std::function<kortex_driver::GetSafetyConfiguration::Response(const kortex_driver::GetSafetyConfiguration::Request&)> GetSafetyConfigurationHandler = nullptr;
        virtual bool GetSafetyConfiguration(kortex_driver::GetSafetyConfiguration::Request  &req, kortex_driver::GetSafetyConfiguration::Response &res) override;
        std::function<kortex_driver::GetSafetyInformation::Response(const kortex_driver::GetSafetyInformation::Request&)> GetSafetyInformationHandler = nullptr;
        virtual bool GetSafetyInformation(kortex_driver::GetSafetyInformation::Request  &req, kortex_driver::GetSafetyInformation::Response &res) override;
        std::function<kortex_driver::GetSafetyEnable::Response(const kortex_driver::GetSafetyEnable::Request&)> GetSafetyEnableHandler = nullptr;
        virtual bool GetSafetyEnable(kortex_driver::GetSafetyEnable::Request  &req, kortex_driver::GetSafetyEnable::Response &res) override;
        std::function<kortex_driver::GetSafetyStatus::Response(const kortex_driver::GetSafetyStatus::Request&)> GetSafetyStatusHandler = nullptr;
        virtual bool GetSafetyStatus(kortex_driver::GetSafetyStatus::Request  &req, kortex_driver::GetSafetyStatus::Response &res) override;
        std::function<kortex_driver::ClearAllSafetyStatus::Response(const kortex_driver::ClearAllSafetyStatus::Request&)> ClearAllSafetyStatusHandler = nullptr;
        virtual bool ClearAllSafetyStatus(kortex_driver::ClearAllSafetyStatus::Request  &req, kortex_driver::ClearAllSafetyStatus::Response &res) override;
        std::function<kortex_driver::ClearSafetyStatus::Response(const kortex_driver::ClearSafetyStatus::Request&)> ClearSafetyStatusHandler = nullptr;
        virtual bool ClearSafetyStatus(kortex_driver::ClearSafetyStatus::Request  &req, kortex_driver::ClearSafetyStatus::Response &res) override;
        std::function<kortex_driver::GetAllSafetyConfiguration::Response(const kortex_driver::GetAllSafetyConfiguration::Request&)> GetAllSafetyConfigurationHandler = nullptr;
        virtual bool GetAllSafetyConfiguration(kortex_driver::GetAllSafetyConfiguration::Request  &req, kortex_driver::GetAllSafetyConfiguration::Response &res) override;
        std::function<kortex_driver::GetAllSafetyInformation::Response(const kortex_driver::GetAllSafetyInformation::Request&)> GetAllSafetyInformationHandler = nullptr;
        virtual bool GetAllSafetyInformation(kortex_driver::GetAllSafetyInformation::Request  &req, kortex_driver::GetAllSafetyInformation::Response &res) override;
        std::function<kortex_driver::ResetSafetyDefaults::Response(const kortex_driver::ResetSafetyDefaults::Request&)> ResetSafetyDefaultsHandler = nullptr;
        virtual bool ResetSafetyDefaults(kortex_driver::ResetSafetyDefaults::Request  &req, kortex_driver::ResetSafetyDefaults::Response &res) override;
        std::function<kortex_driver::OnNotificationSafetyTopic::Response(const kortex_driver::OnNotificationSafetyTopic::Request&)> OnNotificationSafetyTopicHandler = nullptr;
        virtual bool OnNotificationSafetyTopic(kortex_driver::OnNotificationSafetyTopic::Request  &req, kortex_driver::OnNotificationSafetyTopic::Response &res) override;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) override;
        std::function<kortex_driver::ExecuteCalibration::Response(const kortex_driver::ExecuteCalibration::Request&)> ExecuteCalibrationHandler = nullptr;
        virtual bool ExecuteCalibration(kortex_driver::ExecuteCalibration::Request  &req, kortex_driver::ExecuteCalibration::Response &res) override;
        std::function<kortex_driver::GetCalibrationResult::Response(const kortex_driver::GetCalibrationResult::Request&)> GetCalibrationResultHandler = nullptr;
        virtual bool GetCalibrationResult(kortex_driver::GetCalibrationResult::Request  &req, kortex_driver::GetCalibrationResult::Response &res) override;
        std::function<kortex_driver::StopCalibration::Response(const kortex_driver::StopCalibration::Request&)> StopCalibrationHandler = nullptr;
        virtual bool StopCalibration(kortex_driver::StopCalibration::Request  &req, kortex_driver::StopCalibration::Response &res) override;
        std::function<kortex_driver::DeviceConfig_SetCapSenseConfig::Response(const kortex_driver::DeviceConfig_SetCapSenseConfig::Request&)> DeviceConfig_SetCapSenseConfigHandler = nullptr;
        virtual bool DeviceConfig_SetCapSenseConfig(kortex_driver::DeviceConfig_SetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_SetCapSenseConfig::Response &res) override;
        std::function<kortex_driver::DeviceConfig_GetCapSenseConfig::Response(const kortex_driver::DeviceConfig_GetCapSenseConfig::Request&)> DeviceConfig_GetCapSenseConfigHandler = nullptr;
        virtual bool DeviceConfig_GetCapSenseConfig(kortex_driver::DeviceConfig_GetCapSenseConfig::Request  &req, kortex_driver::DeviceConfig_GetCapSenseConfig::Response &res) override;

};
#endif
