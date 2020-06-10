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
 
#ifndef _KORTEX_VISIONCONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_VISIONCONFIG_SERVICES_INTERFACE_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/SetSensorSettings.h"
#include "kortex_driver/GetSensorSettings.h"
#include "kortex_driver/GetOptionValue.h"
#include "kortex_driver/SetOptionValue.h"
#include "kortex_driver/GetOptionInformation.h"
#include "kortex_driver/OnNotificationVisionTopic.h"
#include "kortex_driver/VisionNotification.h"
#include "kortex_driver/DoSensorFocusAction.h"
#include "kortex_driver/GetIntrinsicParameters.h"
#include "kortex_driver/GetIntrinsicParametersProfile.h"
#include "kortex_driver/SetIntrinsicParameters.h"
#include "kortex_driver/GetExtrinsicParameters.h"
#include "kortex_driver/SetExtrinsicParameters.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class IVisionConfigServices
{
    public:
        IVisionConfigServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) = 0;
        virtual bool SetSensorSettings(kortex_driver::SetSensorSettings::Request  &req, kortex_driver::SetSensorSettings::Response &res) = 0;
        virtual bool GetSensorSettings(kortex_driver::GetSensorSettings::Request  &req, kortex_driver::GetSensorSettings::Response &res) = 0;
        virtual bool GetOptionValue(kortex_driver::GetOptionValue::Request  &req, kortex_driver::GetOptionValue::Response &res) = 0;
        virtual bool SetOptionValue(kortex_driver::SetOptionValue::Request  &req, kortex_driver::SetOptionValue::Response &res) = 0;
        virtual bool GetOptionInformation(kortex_driver::GetOptionInformation::Request  &req, kortex_driver::GetOptionInformation::Response &res) = 0;
        virtual bool OnNotificationVisionTopic(kortex_driver::OnNotificationVisionTopic::Request  &req, kortex_driver::OnNotificationVisionTopic::Response &res) = 0;
        virtual void cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif) = 0;
        virtual bool DoSensorFocusAction(kortex_driver::DoSensorFocusAction::Request  &req, kortex_driver::DoSensorFocusAction::Response &res) = 0;
        virtual bool GetIntrinsicParameters(kortex_driver::GetIntrinsicParameters::Request  &req, kortex_driver::GetIntrinsicParameters::Response &res) = 0;
        virtual bool GetIntrinsicParametersProfile(kortex_driver::GetIntrinsicParametersProfile::Request  &req, kortex_driver::GetIntrinsicParametersProfile::Response &res) = 0;
        virtual bool SetIntrinsicParameters(kortex_driver::SetIntrinsicParameters::Request  &req, kortex_driver::SetIntrinsicParameters::Response &res) = 0;
        virtual bool GetExtrinsicParameters(kortex_driver::GetExtrinsicParameters::Request  &req, kortex_driver::GetExtrinsicParameters::Response &res) = 0;
        virtual bool SetExtrinsicParameters(kortex_driver::SetExtrinsicParameters::Request  &req, kortex_driver::SetExtrinsicParameters::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_VisionTopic;
        bool m_is_activated_VisionTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceSetSensorSettings;
	ros::ServiceServer m_serviceGetSensorSettings;
	ros::ServiceServer m_serviceGetOptionValue;
	ros::ServiceServer m_serviceSetOptionValue;
	ros::ServiceServer m_serviceGetOptionInformation;
	ros::ServiceServer m_serviceOnNotificationVisionTopic;
	ros::ServiceServer m_serviceDoSensorFocusAction;
	ros::ServiceServer m_serviceGetIntrinsicParameters;
	ros::ServiceServer m_serviceGetIntrinsicParametersProfile;
	ros::ServiceServer m_serviceSetIntrinsicParameters;
	ros::ServiceServer m_serviceGetExtrinsicParameters;
	ros::ServiceServer m_serviceSetExtrinsicParameters;
};
#endif
