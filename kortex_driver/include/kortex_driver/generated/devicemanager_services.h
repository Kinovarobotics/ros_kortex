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
 
#ifndef _KORTEX_DEVICEMANAGER_SERVICES_H_
#define _KORTEX_DEVICEMANAGER_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <DeviceManager.pb.h>
#include <DeviceManagerClientRpc.h>
#include "kortex_driver/ReadAllDevices.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class DeviceManagerServices
{
    public:
        DeviceManagerServices(ros::NodeHandle& n, Kinova::Api::DeviceManager::DeviceManagerClient* devicemanager, uint32_t device_id, uint32_t timeout_ms);

        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);
        bool ReadAllDevices(kortex_driver::ReadAllDevices::Request  &req, kortex_driver::ReadAllDevices::Response &res);

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::DeviceManager::DeviceManagerClient* m_devicemanager;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceReadAllDevices;
};
#endif
