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
 
#ifndef _KORTEX_ACTUATORCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_ACTUATORCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/actuatorconfig_services_interface.h"

using namespace std;

class ActuatorConfigSimulationServices : public IActuatorConfigServices
{
    public:
        ActuatorConfigSimulationServices(ros::NodeHandle& node_handle);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        std::function<kortex_driver::GetAxisOffsets::Response(const kortex_driver::GetAxisOffsets::Request&)> GetAxisOffsetsHandler = nullptr;
        virtual bool GetAxisOffsets(kortex_driver::GetAxisOffsets::Request  &req, kortex_driver::GetAxisOffsets::Response &res) override;
        std::function<kortex_driver::SetAxisOffsets::Response(const kortex_driver::SetAxisOffsets::Request&)> SetAxisOffsetsHandler = nullptr;
        virtual bool SetAxisOffsets(kortex_driver::SetAxisOffsets::Request  &req, kortex_driver::SetAxisOffsets::Response &res) override;
        std::function<kortex_driver::SetTorqueOffset::Response(const kortex_driver::SetTorqueOffset::Request&)> SetTorqueOffsetHandler = nullptr;
        virtual bool SetTorqueOffset(kortex_driver::SetTorqueOffset::Request  &req, kortex_driver::SetTorqueOffset::Response &res) override;
        std::function<kortex_driver::ActuatorConfig_GetControlMode::Response(const kortex_driver::ActuatorConfig_GetControlMode::Request&)> ActuatorConfig_GetControlModeHandler = nullptr;
        virtual bool ActuatorConfig_GetControlMode(kortex_driver::ActuatorConfig_GetControlMode::Request  &req, kortex_driver::ActuatorConfig_GetControlMode::Response &res) override;
        std::function<kortex_driver::SetControlMode::Response(const kortex_driver::SetControlMode::Request&)> SetControlModeHandler = nullptr;
        virtual bool SetControlMode(kortex_driver::SetControlMode::Request  &req, kortex_driver::SetControlMode::Response &res) override;
        std::function<kortex_driver::GetActivatedControlLoop::Response(const kortex_driver::GetActivatedControlLoop::Request&)> GetActivatedControlLoopHandler = nullptr;
        virtual bool GetActivatedControlLoop(kortex_driver::GetActivatedControlLoop::Request  &req, kortex_driver::GetActivatedControlLoop::Response &res) override;
        std::function<kortex_driver::SetActivatedControlLoop::Response(const kortex_driver::SetActivatedControlLoop::Request&)> SetActivatedControlLoopHandler = nullptr;
        virtual bool SetActivatedControlLoop(kortex_driver::SetActivatedControlLoop::Request  &req, kortex_driver::SetActivatedControlLoop::Response &res) override;
        std::function<kortex_driver::GetControlLoopParameters::Response(const kortex_driver::GetControlLoopParameters::Request&)> GetControlLoopParametersHandler = nullptr;
        virtual bool GetControlLoopParameters(kortex_driver::GetControlLoopParameters::Request  &req, kortex_driver::GetControlLoopParameters::Response &res) override;
        std::function<kortex_driver::SetControlLoopParameters::Response(const kortex_driver::SetControlLoopParameters::Request&)> SetControlLoopParametersHandler = nullptr;
        virtual bool SetControlLoopParameters(kortex_driver::SetControlLoopParameters::Request  &req, kortex_driver::SetControlLoopParameters::Response &res) override;
        std::function<kortex_driver::SelectCustomData::Response(const kortex_driver::SelectCustomData::Request&)> SelectCustomDataHandler = nullptr;
        virtual bool SelectCustomData(kortex_driver::SelectCustomData::Request  &req, kortex_driver::SelectCustomData::Response &res) override;
        std::function<kortex_driver::GetSelectedCustomData::Response(const kortex_driver::GetSelectedCustomData::Request&)> GetSelectedCustomDataHandler = nullptr;
        virtual bool GetSelectedCustomData(kortex_driver::GetSelectedCustomData::Request  &req, kortex_driver::GetSelectedCustomData::Response &res) override;
        std::function<kortex_driver::SetCommandMode::Response(const kortex_driver::SetCommandMode::Request&)> SetCommandModeHandler = nullptr;
        virtual bool SetCommandMode(kortex_driver::SetCommandMode::Request  &req, kortex_driver::SetCommandMode::Response &res) override;
        std::function<kortex_driver::ActuatorConfig_ClearFaults::Response(const kortex_driver::ActuatorConfig_ClearFaults::Request&)> ActuatorConfig_ClearFaultsHandler = nullptr;
        virtual bool ActuatorConfig_ClearFaults(kortex_driver::ActuatorConfig_ClearFaults::Request  &req, kortex_driver::ActuatorConfig_ClearFaults::Response &res) override;
        std::function<kortex_driver::SetServoing::Response(const kortex_driver::SetServoing::Request&)> SetServoingHandler = nullptr;
        virtual bool SetServoing(kortex_driver::SetServoing::Request  &req, kortex_driver::SetServoing::Response &res) override;
        std::function<kortex_driver::MoveToPosition::Response(const kortex_driver::MoveToPosition::Request&)> MoveToPositionHandler = nullptr;
        virtual bool MoveToPosition(kortex_driver::MoveToPosition::Request  &req, kortex_driver::MoveToPosition::Response &res) override;
        std::function<kortex_driver::GetCommandMode::Response(const kortex_driver::GetCommandMode::Request&)> GetCommandModeHandler = nullptr;
        virtual bool GetCommandMode(kortex_driver::GetCommandMode::Request  &req, kortex_driver::GetCommandMode::Response &res) override;
        std::function<kortex_driver::GetServoing::Response(const kortex_driver::GetServoing::Request&)> GetServoingHandler = nullptr;
        virtual bool GetServoing(kortex_driver::GetServoing::Request  &req, kortex_driver::GetServoing::Response &res) override;
        std::function<kortex_driver::GetTorqueOffset::Response(const kortex_driver::GetTorqueOffset::Request&)> GetTorqueOffsetHandler = nullptr;
        virtual bool GetTorqueOffset(kortex_driver::GetTorqueOffset::Request  &req, kortex_driver::GetTorqueOffset::Response &res) override;
        std::function<kortex_driver::SetCoggingFeedforwardMode::Response(const kortex_driver::SetCoggingFeedforwardMode::Request&)> SetCoggingFeedforwardModeHandler = nullptr;
        virtual bool SetCoggingFeedforwardMode(kortex_driver::SetCoggingFeedforwardMode::Request  &req, kortex_driver::SetCoggingFeedforwardMode::Response &res) override;
        std::function<kortex_driver::GetCoggingFeedforwardMode::Response(const kortex_driver::GetCoggingFeedforwardMode::Request&)> GetCoggingFeedforwardModeHandler = nullptr;
        virtual bool GetCoggingFeedforwardMode(kortex_driver::GetCoggingFeedforwardMode::Request  &req, kortex_driver::GetCoggingFeedforwardMode::Response &res) override;

};
#endif
