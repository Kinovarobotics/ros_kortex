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
 
#ifndef _KORTEX_ACTUATORCONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_ACTUATORCONFIG_SERVICES_INTERFACE_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/GetAxisOffsets.h"
#include "kortex_driver/SetAxisOffsets.h"
#include "kortex_driver/SetTorqueOffset.h"
#include "kortex_driver/ActuatorConfig_GetControlMode.h"
#include "kortex_driver/SetControlMode.h"
#include "kortex_driver/GetActivatedControlLoop.h"
#include "kortex_driver/SetActivatedControlLoop.h"
#include "kortex_driver/GetControlLoopParameters.h"
#include "kortex_driver/SetControlLoopParameters.h"
#include "kortex_driver/SelectCustomData.h"
#include "kortex_driver/GetSelectedCustomData.h"
#include "kortex_driver/SetCommandMode.h"
#include "kortex_driver/ActuatorConfig_ClearFaults.h"
#include "kortex_driver/SetServoing.h"
#include "kortex_driver/MoveToPosition.h"
#include "kortex_driver/GetCommandMode.h"
#include "kortex_driver/GetServoing.h"
#include "kortex_driver/GetTorqueOffset.h"
#include "kortex_driver/SetCoggingFeedforwardMode.h"
#include "kortex_driver/GetCoggingFeedforwardMode.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class IActuatorConfigServices
{
    public:
        IActuatorConfigServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) = 0;
        virtual bool GetAxisOffsets(kortex_driver::GetAxisOffsets::Request  &req, kortex_driver::GetAxisOffsets::Response &res) = 0;
        virtual bool SetAxisOffsets(kortex_driver::SetAxisOffsets::Request  &req, kortex_driver::SetAxisOffsets::Response &res) = 0;
        virtual bool SetTorqueOffset(kortex_driver::SetTorqueOffset::Request  &req, kortex_driver::SetTorqueOffset::Response &res) = 0;
        virtual bool ActuatorConfig_GetControlMode(kortex_driver::ActuatorConfig_GetControlMode::Request  &req, kortex_driver::ActuatorConfig_GetControlMode::Response &res) = 0;
        virtual bool SetControlMode(kortex_driver::SetControlMode::Request  &req, kortex_driver::SetControlMode::Response &res) = 0;
        virtual bool GetActivatedControlLoop(kortex_driver::GetActivatedControlLoop::Request  &req, kortex_driver::GetActivatedControlLoop::Response &res) = 0;
        virtual bool SetActivatedControlLoop(kortex_driver::SetActivatedControlLoop::Request  &req, kortex_driver::SetActivatedControlLoop::Response &res) = 0;
        virtual bool GetControlLoopParameters(kortex_driver::GetControlLoopParameters::Request  &req, kortex_driver::GetControlLoopParameters::Response &res) = 0;
        virtual bool SetControlLoopParameters(kortex_driver::SetControlLoopParameters::Request  &req, kortex_driver::SetControlLoopParameters::Response &res) = 0;
        virtual bool SelectCustomData(kortex_driver::SelectCustomData::Request  &req, kortex_driver::SelectCustomData::Response &res) = 0;
        virtual bool GetSelectedCustomData(kortex_driver::GetSelectedCustomData::Request  &req, kortex_driver::GetSelectedCustomData::Response &res) = 0;
        virtual bool SetCommandMode(kortex_driver::SetCommandMode::Request  &req, kortex_driver::SetCommandMode::Response &res) = 0;
        virtual bool ActuatorConfig_ClearFaults(kortex_driver::ActuatorConfig_ClearFaults::Request  &req, kortex_driver::ActuatorConfig_ClearFaults::Response &res) = 0;
        virtual bool SetServoing(kortex_driver::SetServoing::Request  &req, kortex_driver::SetServoing::Response &res) = 0;
        virtual bool MoveToPosition(kortex_driver::MoveToPosition::Request  &req, kortex_driver::MoveToPosition::Response &res) = 0;
        virtual bool GetCommandMode(kortex_driver::GetCommandMode::Request  &req, kortex_driver::GetCommandMode::Response &res) = 0;
        virtual bool GetServoing(kortex_driver::GetServoing::Request  &req, kortex_driver::GetServoing::Response &res) = 0;
        virtual bool GetTorqueOffset(kortex_driver::GetTorqueOffset::Request  &req, kortex_driver::GetTorqueOffset::Response &res) = 0;
        virtual bool SetCoggingFeedforwardMode(kortex_driver::SetCoggingFeedforwardMode::Request  &req, kortex_driver::SetCoggingFeedforwardMode::Response &res) = 0;
        virtual bool GetCoggingFeedforwardMode(kortex_driver::GetCoggingFeedforwardMode::Request  &req, kortex_driver::GetCoggingFeedforwardMode::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceGetAxisOffsets;
	ros::ServiceServer m_serviceSetAxisOffsets;
	ros::ServiceServer m_serviceSetTorqueOffset;
	ros::ServiceServer m_serviceActuatorConfig_GetControlMode;
	ros::ServiceServer m_serviceSetControlMode;
	ros::ServiceServer m_serviceGetActivatedControlLoop;
	ros::ServiceServer m_serviceSetActivatedControlLoop;
	ros::ServiceServer m_serviceGetControlLoopParameters;
	ros::ServiceServer m_serviceSetControlLoopParameters;
	ros::ServiceServer m_serviceSelectCustomData;
	ros::ServiceServer m_serviceGetSelectedCustomData;
	ros::ServiceServer m_serviceSetCommandMode;
	ros::ServiceServer m_serviceActuatorConfig_ClearFaults;
	ros::ServiceServer m_serviceSetServoing;
	ros::ServiceServer m_serviceMoveToPosition;
	ros::ServiceServer m_serviceGetCommandMode;
	ros::ServiceServer m_serviceGetServoing;
	ros::ServiceServer m_serviceGetTorqueOffset;
	ros::ServiceServer m_serviceSetCoggingFeedforwardMode;
	ros::ServiceServer m_serviceGetCoggingFeedforwardMode;
};
#endif
