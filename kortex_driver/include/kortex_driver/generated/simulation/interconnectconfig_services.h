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
 
#ifndef _KORTEX_INTERCONNECTCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_INTERCONNECTCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/interconnectconfig_services_interface.h"

using namespace std;

class InterconnectConfigSimulationServices : public IInterconnectConfigServices
{
    public:
        InterconnectConfigSimulationServices(ros::NodeHandle& node_handle);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        std::function<kortex_driver::GetUARTConfiguration::Response(const kortex_driver::GetUARTConfiguration::Request&)> GetUARTConfigurationHandler = nullptr;
        virtual bool GetUARTConfiguration(kortex_driver::GetUARTConfiguration::Request  &req, kortex_driver::GetUARTConfiguration::Response &res) override;
        std::function<kortex_driver::SetUARTConfiguration::Response(const kortex_driver::SetUARTConfiguration::Request&)> SetUARTConfigurationHandler = nullptr;
        virtual bool SetUARTConfiguration(kortex_driver::SetUARTConfiguration::Request  &req, kortex_driver::SetUARTConfiguration::Response &res) override;
        std::function<kortex_driver::GetEthernetConfiguration::Response(const kortex_driver::GetEthernetConfiguration::Request&)> GetEthernetConfigurationHandler = nullptr;
        virtual bool GetEthernetConfiguration(kortex_driver::GetEthernetConfiguration::Request  &req, kortex_driver::GetEthernetConfiguration::Response &res) override;
        std::function<kortex_driver::SetEthernetConfiguration::Response(const kortex_driver::SetEthernetConfiguration::Request&)> SetEthernetConfigurationHandler = nullptr;
        virtual bool SetEthernetConfiguration(kortex_driver::SetEthernetConfiguration::Request  &req, kortex_driver::SetEthernetConfiguration::Response &res) override;
        std::function<kortex_driver::GetGPIOConfiguration::Response(const kortex_driver::GetGPIOConfiguration::Request&)> GetGPIOConfigurationHandler = nullptr;
        virtual bool GetGPIOConfiguration(kortex_driver::GetGPIOConfiguration::Request  &req, kortex_driver::GetGPIOConfiguration::Response &res) override;
        std::function<kortex_driver::SetGPIOConfiguration::Response(const kortex_driver::SetGPIOConfiguration::Request&)> SetGPIOConfigurationHandler = nullptr;
        virtual bool SetGPIOConfiguration(kortex_driver::SetGPIOConfiguration::Request  &req, kortex_driver::SetGPIOConfiguration::Response &res) override;
        std::function<kortex_driver::GetGPIOState::Response(const kortex_driver::GetGPIOState::Request&)> GetGPIOStateHandler = nullptr;
        virtual bool GetGPIOState(kortex_driver::GetGPIOState::Request  &req, kortex_driver::GetGPIOState::Response &res) override;
        std::function<kortex_driver::SetGPIOState::Response(const kortex_driver::SetGPIOState::Request&)> SetGPIOStateHandler = nullptr;
        virtual bool SetGPIOState(kortex_driver::SetGPIOState::Request  &req, kortex_driver::SetGPIOState::Response &res) override;
        std::function<kortex_driver::GetI2CConfiguration::Response(const kortex_driver::GetI2CConfiguration::Request&)> GetI2CConfigurationHandler = nullptr;
        virtual bool GetI2CConfiguration(kortex_driver::GetI2CConfiguration::Request  &req, kortex_driver::GetI2CConfiguration::Response &res) override;
        std::function<kortex_driver::SetI2CConfiguration::Response(const kortex_driver::SetI2CConfiguration::Request&)> SetI2CConfigurationHandler = nullptr;
        virtual bool SetI2CConfiguration(kortex_driver::SetI2CConfiguration::Request  &req, kortex_driver::SetI2CConfiguration::Response &res) override;
        std::function<kortex_driver::I2CRead::Response(const kortex_driver::I2CRead::Request&)> I2CReadHandler = nullptr;
        virtual bool I2CRead(kortex_driver::I2CRead::Request  &req, kortex_driver::I2CRead::Response &res) override;
        std::function<kortex_driver::I2CReadRegister::Response(const kortex_driver::I2CReadRegister::Request&)> I2CReadRegisterHandler = nullptr;
        virtual bool I2CReadRegister(kortex_driver::I2CReadRegister::Request  &req, kortex_driver::I2CReadRegister::Response &res) override;
        std::function<kortex_driver::I2CWrite::Response(const kortex_driver::I2CWrite::Request&)> I2CWriteHandler = nullptr;
        virtual bool I2CWrite(kortex_driver::I2CWrite::Request  &req, kortex_driver::I2CWrite::Response &res) override;
        std::function<kortex_driver::I2CWriteRegister::Response(const kortex_driver::I2CWriteRegister::Request&)> I2CWriteRegisterHandler = nullptr;
        virtual bool I2CWriteRegister(kortex_driver::I2CWriteRegister::Request  &req, kortex_driver::I2CWriteRegister::Response &res) override;

};
#endif
