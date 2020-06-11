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
 
#ifndef _KORTEX_VISIONCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_VISIONCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/visionconfig_services_interface.h"

using namespace std;

class VisionConfigSimulationServices : public IVisionConfigServices
{
    public:
        VisionConfigSimulationServices(ros::NodeHandle& node_handle);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        std::function<kortex_driver::SetSensorSettings::Response(const kortex_driver::SetSensorSettings::Request&)> SetSensorSettingsHandler = nullptr;
        virtual bool SetSensorSettings(kortex_driver::SetSensorSettings::Request  &req, kortex_driver::SetSensorSettings::Response &res) override;
        std::function<kortex_driver::GetSensorSettings::Response(const kortex_driver::GetSensorSettings::Request&)> GetSensorSettingsHandler = nullptr;
        virtual bool GetSensorSettings(kortex_driver::GetSensorSettings::Request  &req, kortex_driver::GetSensorSettings::Response &res) override;
        std::function<kortex_driver::GetOptionValue::Response(const kortex_driver::GetOptionValue::Request&)> GetOptionValueHandler = nullptr;
        virtual bool GetOptionValue(kortex_driver::GetOptionValue::Request  &req, kortex_driver::GetOptionValue::Response &res) override;
        std::function<kortex_driver::SetOptionValue::Response(const kortex_driver::SetOptionValue::Request&)> SetOptionValueHandler = nullptr;
        virtual bool SetOptionValue(kortex_driver::SetOptionValue::Request  &req, kortex_driver::SetOptionValue::Response &res) override;
        std::function<kortex_driver::GetOptionInformation::Response(const kortex_driver::GetOptionInformation::Request&)> GetOptionInformationHandler = nullptr;
        virtual bool GetOptionInformation(kortex_driver::GetOptionInformation::Request  &req, kortex_driver::GetOptionInformation::Response &res) override;
        std::function<kortex_driver::OnNotificationVisionTopic::Response(const kortex_driver::OnNotificationVisionTopic::Request&)> OnNotificationVisionTopicHandler = nullptr;
        virtual bool OnNotificationVisionTopic(kortex_driver::OnNotificationVisionTopic::Request  &req, kortex_driver::OnNotificationVisionTopic::Response &res) override;
        virtual void cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif) override;
        std::function<kortex_driver::DoSensorFocusAction::Response(const kortex_driver::DoSensorFocusAction::Request&)> DoSensorFocusActionHandler = nullptr;
        virtual bool DoSensorFocusAction(kortex_driver::DoSensorFocusAction::Request  &req, kortex_driver::DoSensorFocusAction::Response &res) override;
        std::function<kortex_driver::GetIntrinsicParameters::Response(const kortex_driver::GetIntrinsicParameters::Request&)> GetIntrinsicParametersHandler = nullptr;
        virtual bool GetIntrinsicParameters(kortex_driver::GetIntrinsicParameters::Request  &req, kortex_driver::GetIntrinsicParameters::Response &res) override;
        std::function<kortex_driver::GetIntrinsicParametersProfile::Response(const kortex_driver::GetIntrinsicParametersProfile::Request&)> GetIntrinsicParametersProfileHandler = nullptr;
        virtual bool GetIntrinsicParametersProfile(kortex_driver::GetIntrinsicParametersProfile::Request  &req, kortex_driver::GetIntrinsicParametersProfile::Response &res) override;
        std::function<kortex_driver::SetIntrinsicParameters::Response(const kortex_driver::SetIntrinsicParameters::Request&)> SetIntrinsicParametersHandler = nullptr;
        virtual bool SetIntrinsicParameters(kortex_driver::SetIntrinsicParameters::Request  &req, kortex_driver::SetIntrinsicParameters::Response &res) override;
        std::function<kortex_driver::GetExtrinsicParameters::Response(const kortex_driver::GetExtrinsicParameters::Request&)> GetExtrinsicParametersHandler = nullptr;
        virtual bool GetExtrinsicParameters(kortex_driver::GetExtrinsicParameters::Request  &req, kortex_driver::GetExtrinsicParameters::Response &res) override;
        std::function<kortex_driver::SetExtrinsicParameters::Response(const kortex_driver::SetExtrinsicParameters::Request&)> SetExtrinsicParametersHandler = nullptr;
        virtual bool SetExtrinsicParameters(kortex_driver::SetExtrinsicParameters::Request  &req, kortex_driver::SetExtrinsicParameters::Response &res) override;

};
#endif
