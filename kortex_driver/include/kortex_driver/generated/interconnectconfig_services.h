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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_SERVICES_H_
#define _KORTEX_INTERCONNECTCONFIG_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <InterconnectConfig.pb.h>
#include <InterconnectConfigClientRpc.h>
#include "kortex_driver/GetUARTConfiguration.h"
#include "kortex_driver/SetUARTConfiguration.h"
#include "kortex_driver/GetEthernetConfiguration.h"
#include "kortex_driver/SetEthernetConfiguration.h"
#include "kortex_driver/GetGPIOConfiguration.h"
#include "kortex_driver/SetGPIOConfiguration.h"
#include "kortex_driver/GetGPIOState.h"
#include "kortex_driver/SetGPIOState.h"
#include "kortex_driver/GetI2CConfiguration.h"
#include "kortex_driver/SetI2CConfiguration.h"
#include "kortex_driver/I2CRead.h"
#include "kortex_driver/I2CReadRegister.h"
#include "kortex_driver/I2CWrite.h"
#include "kortex_driver/I2CWriteRegister.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class InterconnectConfigServices
{
    public:
        InterconnectConfigServices(ros::NodeHandle& n, Kinova::Api::InterconnectConfig::InterconnectConfigClient* interconnectconfig, uint32_t device_id, uint32_t timeout_ms);

        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);
        bool GetUARTConfiguration(kortex_driver::GetUARTConfiguration::Request  &req, kortex_driver::GetUARTConfiguration::Response &res);
        bool SetUARTConfiguration(kortex_driver::SetUARTConfiguration::Request  &req, kortex_driver::SetUARTConfiguration::Response &res);
        bool GetEthernetConfiguration(kortex_driver::GetEthernetConfiguration::Request  &req, kortex_driver::GetEthernetConfiguration::Response &res);
        bool SetEthernetConfiguration(kortex_driver::SetEthernetConfiguration::Request  &req, kortex_driver::SetEthernetConfiguration::Response &res);
        bool GetGPIOConfiguration(kortex_driver::GetGPIOConfiguration::Request  &req, kortex_driver::GetGPIOConfiguration::Response &res);
        bool SetGPIOConfiguration(kortex_driver::SetGPIOConfiguration::Request  &req, kortex_driver::SetGPIOConfiguration::Response &res);
        bool GetGPIOState(kortex_driver::GetGPIOState::Request  &req, kortex_driver::GetGPIOState::Response &res);
        bool SetGPIOState(kortex_driver::SetGPIOState::Request  &req, kortex_driver::SetGPIOState::Response &res);
        bool GetI2CConfiguration(kortex_driver::GetI2CConfiguration::Request  &req, kortex_driver::GetI2CConfiguration::Response &res);
        bool SetI2CConfiguration(kortex_driver::SetI2CConfiguration::Request  &req, kortex_driver::SetI2CConfiguration::Response &res);
        bool I2CRead(kortex_driver::I2CRead::Request  &req, kortex_driver::I2CRead::Response &res);
        bool I2CReadRegister(kortex_driver::I2CReadRegister::Request  &req, kortex_driver::I2CReadRegister::Response &res);
        bool I2CWrite(kortex_driver::I2CWrite::Request  &req, kortex_driver::I2CWrite::Response &res);
        bool I2CWriteRegister(kortex_driver::I2CWriteRegister::Request  &req, kortex_driver::I2CWriteRegister::Response &res);

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::InterconnectConfig::InterconnectConfigClient* m_interconnectconfig;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceGetUARTConfiguration;
	ros::ServiceServer m_serviceSetUARTConfiguration;
	ros::ServiceServer m_serviceGetEthernetConfiguration;
	ros::ServiceServer m_serviceSetEthernetConfiguration;
	ros::ServiceServer m_serviceGetGPIOConfiguration;
	ros::ServiceServer m_serviceSetGPIOConfiguration;
	ros::ServiceServer m_serviceGetGPIOState;
	ros::ServiceServer m_serviceSetGPIOState;
	ros::ServiceServer m_serviceGetI2CConfiguration;
	ros::ServiceServer m_serviceSetI2CConfiguration;
	ros::ServiceServer m_serviceI2CRead;
	ros::ServiceServer m_serviceI2CReadRegister;
	ros::ServiceServer m_serviceI2CWrite;
	ros::ServiceServer m_serviceI2CWriteRegister;
};
#endif
