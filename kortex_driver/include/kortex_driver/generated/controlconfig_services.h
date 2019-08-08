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
 
#ifndef _KORTEX_CONTROLCONFIG_SERVICES_H_
#define _KORTEX_CONTROLCONFIG_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>
#include <ControlConfigClientRpc.h>
#include "kortex_driver/SetGravityVector.h"
#include "kortex_driver/GetGravityVector.h"
#include "kortex_driver/SetPayloadInformation.h"
#include "kortex_driver/GetPayloadInformation.h"
#include "kortex_driver/SetToolConfiguration.h"
#include "kortex_driver/GetToolConfiguration.h"
#include "kortex_driver/OnNotificationControlConfigurationTopic.h"
#include "kortex_driver/ControlConfigurationNotification.h"
#include "kortex_driver/ControlConfig_Unsubscribe.h"
#include "kortex_driver/SetCartesianReferenceFrame.h"
#include "kortex_driver/GetCartesianReferenceFrame.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class ControlConfigServices
{
    public:
        ControlConfigServices(ros::NodeHandle& n, Kinova::Api::ControlConfig::ControlConfigClient* controlconfig, uint32_t device_id, uint32_t timeout_ms);

        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);
        bool SetGravityVector(kortex_driver::SetGravityVector::Request  &req, kortex_driver::SetGravityVector::Response &res);
        bool GetGravityVector(kortex_driver::GetGravityVector::Request  &req, kortex_driver::GetGravityVector::Response &res);
        bool SetPayloadInformation(kortex_driver::SetPayloadInformation::Request  &req, kortex_driver::SetPayloadInformation::Response &res);
        bool GetPayloadInformation(kortex_driver::GetPayloadInformation::Request  &req, kortex_driver::GetPayloadInformation::Response &res);
        bool SetToolConfiguration(kortex_driver::SetToolConfiguration::Request  &req, kortex_driver::SetToolConfiguration::Response &res);
        bool GetToolConfiguration(kortex_driver::GetToolConfiguration::Request  &req, kortex_driver::GetToolConfiguration::Response &res);
        bool OnNotificationControlConfigurationTopic(kortex_driver::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::OnNotificationControlConfigurationTopic::Response &res);
        void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif);
        bool ControlConfig_Unsubscribe(kortex_driver::ControlConfig_Unsubscribe::Request  &req, kortex_driver::ControlConfig_Unsubscribe::Response &res);
        bool SetCartesianReferenceFrame(kortex_driver::SetCartesianReferenceFrame::Request  &req, kortex_driver::SetCartesianReferenceFrame::Response &res);
        bool GetCartesianReferenceFrame(kortex_driver::GetCartesianReferenceFrame::Request  &req, kortex_driver::GetCartesianReferenceFrame::Response &res);

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::ControlConfig::ControlConfigClient* m_controlconfig;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_ControlConfigurationTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceSetGravityVector;
	ros::ServiceServer m_serviceGetGravityVector;
	ros::ServiceServer m_serviceSetPayloadInformation;
	ros::ServiceServer m_serviceGetPayloadInformation;
	ros::ServiceServer m_serviceSetToolConfiguration;
	ros::ServiceServer m_serviceGetToolConfiguration;
	ros::ServiceServer m_serviceOnNotificationControlConfigurationTopic;
	ros::ServiceServer m_serviceControlConfig_Unsubscribe;
	ros::ServiceServer m_serviceSetCartesianReferenceFrame;
	ros::ServiceServer m_serviceGetCartesianReferenceFrame;
};
#endif
