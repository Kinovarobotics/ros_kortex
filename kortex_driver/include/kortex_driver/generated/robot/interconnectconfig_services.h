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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_ROBOT_SERVICES_H_
#define _KORTEX_INTERCONNECTCONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/interconnectconfig_services_interface.h"

#include <InterconnectConfig.pb.h>
#include <InterconnectConfigClientRpc.h>

using namespace std;

class InterconnectConfigRobotServices : public IInterconnectConfigServices
{
    public:
        InterconnectConfigRobotServices(ros::NodeHandle& node_handle, Kinova::Api::InterconnectConfig::InterconnectConfigClient* interconnectconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        virtual bool GetUARTConfiguration(kortex_driver::GetUARTConfiguration::Request  &req, kortex_driver::GetUARTConfiguration::Response &res) override;
        virtual bool SetUARTConfiguration(kortex_driver::SetUARTConfiguration::Request  &req, kortex_driver::SetUARTConfiguration::Response &res) override;
        virtual bool GetEthernetConfiguration(kortex_driver::GetEthernetConfiguration::Request  &req, kortex_driver::GetEthernetConfiguration::Response &res) override;
        virtual bool SetEthernetConfiguration(kortex_driver::SetEthernetConfiguration::Request  &req, kortex_driver::SetEthernetConfiguration::Response &res) override;
        virtual bool GetGPIOConfiguration(kortex_driver::GetGPIOConfiguration::Request  &req, kortex_driver::GetGPIOConfiguration::Response &res) override;
        virtual bool SetGPIOConfiguration(kortex_driver::SetGPIOConfiguration::Request  &req, kortex_driver::SetGPIOConfiguration::Response &res) override;
        virtual bool GetGPIOState(kortex_driver::GetGPIOState::Request  &req, kortex_driver::GetGPIOState::Response &res) override;
        virtual bool SetGPIOState(kortex_driver::SetGPIOState::Request  &req, kortex_driver::SetGPIOState::Response &res) override;
        virtual bool GetI2CConfiguration(kortex_driver::GetI2CConfiguration::Request  &req, kortex_driver::GetI2CConfiguration::Response &res) override;
        virtual bool SetI2CConfiguration(kortex_driver::SetI2CConfiguration::Request  &req, kortex_driver::SetI2CConfiguration::Response &res) override;
        virtual bool I2CRead(kortex_driver::I2CRead::Request  &req, kortex_driver::I2CRead::Response &res) override;
        virtual bool I2CReadRegister(kortex_driver::I2CReadRegister::Request  &req, kortex_driver::I2CReadRegister::Response &res) override;
        virtual bool I2CWrite(kortex_driver::I2CWrite::Request  &req, kortex_driver::I2CWrite::Response &res) override;
        virtual bool I2CWriteRegister(kortex_driver::I2CWriteRegister::Request  &req, kortex_driver::I2CWriteRegister::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::InterconnectConfig::InterconnectConfigClient* m_interconnectconfig;
};
#endif
