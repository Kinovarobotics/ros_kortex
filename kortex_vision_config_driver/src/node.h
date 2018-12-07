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
#include <VisionConfig.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>
#include <VisionConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include "kortex_vision_config_driver/SetSensorSettings.h"
#include "kortex_vision_config_driver/GetSensorSettings.h"
#include "kortex_vision_config_driver/GetOptionValue.h"
#include "kortex_vision_config_driver/SetOptionValue.h"
#include "kortex_vision_config_driver/GetOptionInformation.h"
#include "kortex_vision_config_driver/VisionTopic.h"
#include "kortex_vision_config_driver/DoSensorFocusAction.h"
#include "kortex_vision_config_driver/GetIntrinsicParameters.h"
#include "kortex_vision_config_driver/KortexError.h"
#include "kortex_vision_config_driver/SetDeviceID.h"
#include "kortex_vision_config_driver/SetApiOptions.h"

#include "kortex_vision_config_driver/ApiOptions.h"

using namespace std;
using namespace Kinova::Api;
using namespace Kinova::Api::Common;
using namespace Kinova::Api::VisionConfig;

class VisionConfig_Services
{
    public:
        VisionConfig_Services(char* ip, ros::NodeHandle& n, uint32_t device_id);
        bool SetDeviceID(kortex_vision_config_driver::SetDeviceID::Request  &req, kortex_vision_config_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_vision_config_driver::SetApiOptions::Request  &req, kortex_vision_config_driver::SetApiOptions::Response &res);


        bool SetSensorSettings(kortex_vision_config_driver::SetSensorSettings::Request  &req, kortex_vision_config_driver::SetSensorSettings::Response &res);
        bool GetSensorSettings(kortex_vision_config_driver::GetSensorSettings::Request  &req, kortex_vision_config_driver::GetSensorSettings::Response &res);
        bool GetOptionValue(kortex_vision_config_driver::GetOptionValue::Request  &req, kortex_vision_config_driver::GetOptionValue::Response &res);
        bool SetOptionValue(kortex_vision_config_driver::SetOptionValue::Request  &req, kortex_vision_config_driver::SetOptionValue::Response &res);
        bool GetOptionInformation(kortex_vision_config_driver::GetOptionInformation::Request  &req, kortex_vision_config_driver::GetOptionInformation::Response &res);
        bool OnNotificationVisionTopic(kortex_vision_config_driver::VisionTopic::Request  &req, kortex_vision_config_driver::VisionTopic::Response &res);
        void cb_VisionTopic(VisionNotification notif);
        bool DoSensorFocusAction(kortex_vision_config_driver::DoSensorFocusAction::Request  &req, kortex_vision_config_driver::DoSensorFocusAction::Response &res);
        bool GetIntrinsicParameters(kortex_vision_config_driver::GetIntrinsicParameters::Request  &req, kortex_vision_config_driver::GetIntrinsicParameters::Response &res);


private:
    	TransportClientUdp* m_transport;
    	RouterClient*       m_router;
        
        VisionConfigClient*   m_visionconfig;
        uint32_t m_CurrentDeviceID;
        RouterClientSendOptions m_apiOptions;

        SessionManager* m_SessionManager;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_VisionTopic;
};
#endif
