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
 
#ifndef _KORTEX_ACTUATORCONFIG_ROBOT_SERVICES_H_
#define _KORTEX_ACTUATORCONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/actuatorconfig_services_interface.h"

#include <ActuatorConfig.pb.h>
#include <ActuatorConfigClientRpc.h>

using namespace std;

class ActuatorConfigRobotServices : public IActuatorConfigServices
{
    public:
        ActuatorConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::ActuatorConfig::ActuatorConfigClient* actuatorconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        virtual bool GetAxisOffsets(kortex_driver::GetAxisOffsets::Request  &req, kortex_driver::GetAxisOffsets::Response &res) override;
        virtual bool SetAxisOffsets(kortex_driver::SetAxisOffsets::Request  &req, kortex_driver::SetAxisOffsets::Response &res) override;
        virtual bool SetTorqueOffset(kortex_driver::SetTorqueOffset::Request  &req, kortex_driver::SetTorqueOffset::Response &res) override;
        virtual bool ActuatorConfig_GetControlMode(kortex_driver::ActuatorConfig_GetControlMode::Request  &req, kortex_driver::ActuatorConfig_GetControlMode::Response &res) override;
        virtual bool SetControlMode(kortex_driver::SetControlMode::Request  &req, kortex_driver::SetControlMode::Response &res) override;
        virtual bool GetActivatedControlLoop(kortex_driver::GetActivatedControlLoop::Request  &req, kortex_driver::GetActivatedControlLoop::Response &res) override;
        virtual bool SetActivatedControlLoop(kortex_driver::SetActivatedControlLoop::Request  &req, kortex_driver::SetActivatedControlLoop::Response &res) override;
        virtual bool GetControlLoopParameters(kortex_driver::GetControlLoopParameters::Request  &req, kortex_driver::GetControlLoopParameters::Response &res) override;
        virtual bool SetControlLoopParameters(kortex_driver::SetControlLoopParameters::Request  &req, kortex_driver::SetControlLoopParameters::Response &res) override;
        virtual bool SelectCustomData(kortex_driver::SelectCustomData::Request  &req, kortex_driver::SelectCustomData::Response &res) override;
        virtual bool GetSelectedCustomData(kortex_driver::GetSelectedCustomData::Request  &req, kortex_driver::GetSelectedCustomData::Response &res) override;
        virtual bool SetCommandMode(kortex_driver::SetCommandMode::Request  &req, kortex_driver::SetCommandMode::Response &res) override;
        virtual bool ActuatorConfig_ClearFaults(kortex_driver::ActuatorConfig_ClearFaults::Request  &req, kortex_driver::ActuatorConfig_ClearFaults::Response &res) override;
        virtual bool SetServoing(kortex_driver::SetServoing::Request  &req, kortex_driver::SetServoing::Response &res) override;
        virtual bool MoveToPosition(kortex_driver::MoveToPosition::Request  &req, kortex_driver::MoveToPosition::Response &res) override;
        virtual bool GetCommandMode(kortex_driver::GetCommandMode::Request  &req, kortex_driver::GetCommandMode::Response &res) override;
        virtual bool GetServoing(kortex_driver::GetServoing::Request  &req, kortex_driver::GetServoing::Response &res) override;
        virtual bool GetTorqueOffset(kortex_driver::GetTorqueOffset::Request  &req, kortex_driver::GetTorqueOffset::Response &res) override;
        virtual bool SetCoggingFeedforwardMode(kortex_driver::SetCoggingFeedforwardMode::Request  &req, kortex_driver::SetCoggingFeedforwardMode::Response &res) override;
        virtual bool GetCoggingFeedforwardMode(kortex_driver::GetCoggingFeedforwardMode::Request  &req, kortex_driver::GetCoggingFeedforwardMode::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::ActuatorConfig::ActuatorConfigClient* m_actuatorconfig;
};
#endif
