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
#include <ActuatorConfig.pb.h>
#include <ActuatorCyclic.pb.h>

#include <KDetailedException.h>
#include <HeaderInfo.h>

#include <TransportClientUdp.h>
#include <RouterClient.h>
#include <ActuatorConfigClientRpc.h>
#include <ActuatorCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include "kortex_actuator_driver/GetAxisOffsets.h"
#include "kortex_actuator_driver/SetAxisOffsets.h"
#include "kortex_actuator_driver/ReadTorqueCalibration.h"
#include "kortex_actuator_driver/WriteTorqueCalibration.h"
#include "kortex_actuator_driver/SetTorqueOffset.h"
#include "kortex_actuator_driver/GetControlMode.h"
#include "kortex_actuator_driver/SetControlMode.h"
#include "kortex_actuator_driver/GetActivatedControlLoop.h"
#include "kortex_actuator_driver/SetActivatedControlLoop.h"
#include "kortex_actuator_driver/GetVectorDriveParameters.h"
#include "kortex_actuator_driver/SetVectorDriveParameters.h"
#include "kortex_actuator_driver/GetEncoderDerivativeParameters.h"
#include "kortex_actuator_driver/SetEncoderDerivativeParameters.h"
#include "kortex_actuator_driver/GetControlLoopParameters.h"
#include "kortex_actuator_driver/SetControlLoopParameters.h"
#include "kortex_actuator_driver/StartFrequencyResponse.h"
#include "kortex_actuator_driver/StopFrequencyResponse.h"
#include "kortex_actuator_driver/StartStepResponse.h"
#include "kortex_actuator_driver/StopStepResponse.h"
#include "kortex_actuator_driver/StartRampResponse.h"
#include "kortex_actuator_driver/StopRampResponse.h"
#include "kortex_actuator_driver/SelectCustomData.h"
#include "kortex_actuator_driver/GetSelectedCustomData.h"
#include "kortex_actuator_driver/SetCommandMode.h"
#include "kortex_actuator_driver/ClearFaults.h"
#include "kortex_actuator_driver/SetServoing.h"
#include "kortex_actuator_driver/MoveToPosition.h"
#include "kortex_actuator_driver/GetCommandMode.h"
#include "kortex_actuator_driver/GetServoing.h"
#include "kortex_actuator_driver/GetTorqueOffset.h"
#include "kortex_actuator_driver/Refresh.h"
#include "kortex_actuator_driver/RefreshCommand.h"
#include "kortex_actuator_driver/RefreshFeedback.h"
#include "kortex_actuator_driver/RefreshCustomData.h"
#include "kortex_actuator_driver/KortexError.h"
#include "kortex_actuator_driver/SetDeviceID.h"
#include "kortex_actuator_driver/SetApiOptions.h"
#include "kortex_actuator_driver/SetCyclicStatus.h"
#include "kortex_actuator_driver/GetCyclicStatus.h"
#include "kortex_actuator_driver/ApiOptions.h"
#include "kortex_actuator_driver/CyclicStatus.h"

using namespace std;
using namespace Kinova::Api;
using namespace Kinova::Api::Common;
using namespace Kinova::Api::ActuatorConfig;
using namespace Kinova::Api::ActuatorCyclic;

class Actuator_Services
{
    public:
        Actuator_Services(char* ip, ros::NodeHandle& n, uint32_t device_id);
        bool SetDeviceID(kortex_actuator_driver::SetDeviceID::Request  &req, kortex_actuator_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_actuator_driver::SetApiOptions::Request  &req, kortex_actuator_driver::SetApiOptions::Response &res);
        bool GetCyclicStatus(kortex_actuator_driver::GetCyclicStatus::Request  &req, kortex_actuator_driver::GetCyclicStatus::Response &res);
        bool SetCyclicStatus(kortex_actuator_driver::SetCyclicStatus::Request  &req, kortex_actuator_driver::SetCyclicStatus::Response &res);
        bool IsCyclicActive();


        bool GetAxisOffsets(kortex_actuator_driver::GetAxisOffsets::Request  &req, kortex_actuator_driver::GetAxisOffsets::Response &res);
        bool SetAxisOffsets(kortex_actuator_driver::SetAxisOffsets::Request  &req, kortex_actuator_driver::SetAxisOffsets::Response &res);
        bool ReadTorqueCalibration(kortex_actuator_driver::ReadTorqueCalibration::Request  &req, kortex_actuator_driver::ReadTorqueCalibration::Response &res);
        bool WriteTorqueCalibration(kortex_actuator_driver::WriteTorqueCalibration::Request  &req, kortex_actuator_driver::WriteTorqueCalibration::Response &res);
        bool SetTorqueOffset(kortex_actuator_driver::SetTorqueOffset::Request  &req, kortex_actuator_driver::SetTorqueOffset::Response &res);
        bool GetControlMode(kortex_actuator_driver::GetControlMode::Request  &req, kortex_actuator_driver::GetControlMode::Response &res);
        bool SetControlMode(kortex_actuator_driver::SetControlMode::Request  &req, kortex_actuator_driver::SetControlMode::Response &res);
        bool GetActivatedControlLoop(kortex_actuator_driver::GetActivatedControlLoop::Request  &req, kortex_actuator_driver::GetActivatedControlLoop::Response &res);
        bool SetActivatedControlLoop(kortex_actuator_driver::SetActivatedControlLoop::Request  &req, kortex_actuator_driver::SetActivatedControlLoop::Response &res);
        bool GetVectorDriveParameters(kortex_actuator_driver::GetVectorDriveParameters::Request  &req, kortex_actuator_driver::GetVectorDriveParameters::Response &res);
        bool SetVectorDriveParameters(kortex_actuator_driver::SetVectorDriveParameters::Request  &req, kortex_actuator_driver::SetVectorDriveParameters::Response &res);
        bool GetEncoderDerivativeParameters(kortex_actuator_driver::GetEncoderDerivativeParameters::Request  &req, kortex_actuator_driver::GetEncoderDerivativeParameters::Response &res);
        bool SetEncoderDerivativeParameters(kortex_actuator_driver::SetEncoderDerivativeParameters::Request  &req, kortex_actuator_driver::SetEncoderDerivativeParameters::Response &res);
        bool GetControlLoopParameters(kortex_actuator_driver::GetControlLoopParameters::Request  &req, kortex_actuator_driver::GetControlLoopParameters::Response &res);
        bool SetControlLoopParameters(kortex_actuator_driver::SetControlLoopParameters::Request  &req, kortex_actuator_driver::SetControlLoopParameters::Response &res);
        bool StartFrequencyResponse(kortex_actuator_driver::StartFrequencyResponse::Request  &req, kortex_actuator_driver::StartFrequencyResponse::Response &res);
        bool StopFrequencyResponse(kortex_actuator_driver::StopFrequencyResponse::Request  &req, kortex_actuator_driver::StopFrequencyResponse::Response &res);
        bool StartStepResponse(kortex_actuator_driver::StartStepResponse::Request  &req, kortex_actuator_driver::StartStepResponse::Response &res);
        bool StopStepResponse(kortex_actuator_driver::StopStepResponse::Request  &req, kortex_actuator_driver::StopStepResponse::Response &res);
        bool StartRampResponse(kortex_actuator_driver::StartRampResponse::Request  &req, kortex_actuator_driver::StartRampResponse::Response &res);
        bool StopRampResponse(kortex_actuator_driver::StopRampResponse::Request  &req, kortex_actuator_driver::StopRampResponse::Response &res);
        bool SelectCustomData(kortex_actuator_driver::SelectCustomData::Request  &req, kortex_actuator_driver::SelectCustomData::Response &res);
        bool GetSelectedCustomData(kortex_actuator_driver::GetSelectedCustomData::Request  &req, kortex_actuator_driver::GetSelectedCustomData::Response &res);
        bool SetCommandMode(kortex_actuator_driver::SetCommandMode::Request  &req, kortex_actuator_driver::SetCommandMode::Response &res);
        bool ClearFaults(kortex_actuator_driver::ClearFaults::Request  &req, kortex_actuator_driver::ClearFaults::Response &res);
        bool SetServoing(kortex_actuator_driver::SetServoing::Request  &req, kortex_actuator_driver::SetServoing::Response &res);
        bool MoveToPosition(kortex_actuator_driver::MoveToPosition::Request  &req, kortex_actuator_driver::MoveToPosition::Response &res);
        bool GetCommandMode(kortex_actuator_driver::GetCommandMode::Request  &req, kortex_actuator_driver::GetCommandMode::Response &res);
        bool GetServoing(kortex_actuator_driver::GetServoing::Request  &req, kortex_actuator_driver::GetServoing::Response &res);
        bool GetTorqueOffset(kortex_actuator_driver::GetTorqueOffset::Request  &req, kortex_actuator_driver::GetTorqueOffset::Response &res);

        bool Refresh(kortex_actuator_driver::Refresh::Request  &req, kortex_actuator_driver::Refresh::Response &res);
        bool RefreshCommand(kortex_actuator_driver::RefreshCommand::Request  &req, kortex_actuator_driver::RefreshCommand::Response &res);
        bool RefreshFeedback(kortex_actuator_driver::RefreshFeedback::Request  &req, kortex_actuator_driver::RefreshFeedback::Response &res);
        bool RefreshCustomData(kortex_actuator_driver::RefreshCustomData::Request  &req, kortex_actuator_driver::RefreshCustomData::Response &res);


private:
    	TransportClientUdp* m_transport;
    	RouterClient*       m_router;
        
        ActuatorConfigClient*   m_actuatorconfig;
        ActuatorCyclicClient*   m_actuatorcyclic;
        uint32_t m_CurrentDeviceID;
        RouterClientSendOptions m_apiOptions;

        SessionManager* m_SessionManager;
        bool m_cyclicActive = false;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
};
#endif
