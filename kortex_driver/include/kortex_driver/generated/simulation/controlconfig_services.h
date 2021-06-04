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
 
#ifndef _KORTEX_CONTROLCONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_CONTROLCONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/controlconfig_services_interface.h"

using namespace std;

class ControlConfigSimulationServices : public IControlConfigServices
{
    public:
        ControlConfigSimulationServices(ros::NodeHandle& node_handle);

        virtual bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res) override;
        std::function<kortex_driver::SetGravityVector::Response(const kortex_driver::SetGravityVector::Request&)> SetGravityVectorHandler = nullptr;
        virtual bool SetGravityVector(kortex_driver::SetGravityVector::Request  &req, kortex_driver::SetGravityVector::Response &res) override;
        std::function<kortex_driver::GetGravityVector::Response(const kortex_driver::GetGravityVector::Request&)> GetGravityVectorHandler = nullptr;
        virtual bool GetGravityVector(kortex_driver::GetGravityVector::Request  &req, kortex_driver::GetGravityVector::Response &res) override;
        std::function<kortex_driver::SetPayloadInformation::Response(const kortex_driver::SetPayloadInformation::Request&)> SetPayloadInformationHandler = nullptr;
        virtual bool SetPayloadInformation(kortex_driver::SetPayloadInformation::Request  &req, kortex_driver::SetPayloadInformation::Response &res) override;
        std::function<kortex_driver::GetPayloadInformation::Response(const kortex_driver::GetPayloadInformation::Request&)> GetPayloadInformationHandler = nullptr;
        virtual bool GetPayloadInformation(kortex_driver::GetPayloadInformation::Request  &req, kortex_driver::GetPayloadInformation::Response &res) override;
        std::function<kortex_driver::SetToolConfiguration::Response(const kortex_driver::SetToolConfiguration::Request&)> SetToolConfigurationHandler = nullptr;
        virtual bool SetToolConfiguration(kortex_driver::SetToolConfiguration::Request  &req, kortex_driver::SetToolConfiguration::Response &res) override;
        std::function<kortex_driver::GetToolConfiguration::Response(const kortex_driver::GetToolConfiguration::Request&)> GetToolConfigurationHandler = nullptr;
        virtual bool GetToolConfiguration(kortex_driver::GetToolConfiguration::Request  &req, kortex_driver::GetToolConfiguration::Response &res) override;
        std::function<kortex_driver::OnNotificationControlConfigurationTopic::Response(const kortex_driver::OnNotificationControlConfigurationTopic::Request&)> OnNotificationControlConfigurationTopicHandler = nullptr;
        virtual bool OnNotificationControlConfigurationTopic(kortex_driver::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::OnNotificationControlConfigurationTopic::Response &res) override;
        virtual void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif) override;
        std::function<kortex_driver::ControlConfig_Unsubscribe::Response(const kortex_driver::ControlConfig_Unsubscribe::Request&)> ControlConfig_UnsubscribeHandler = nullptr;
        virtual bool ControlConfig_Unsubscribe(kortex_driver::ControlConfig_Unsubscribe::Request  &req, kortex_driver::ControlConfig_Unsubscribe::Response &res) override;
        std::function<kortex_driver::SetCartesianReferenceFrame::Response(const kortex_driver::SetCartesianReferenceFrame::Request&)> SetCartesianReferenceFrameHandler = nullptr;
        virtual bool SetCartesianReferenceFrame(kortex_driver::SetCartesianReferenceFrame::Request  &req, kortex_driver::SetCartesianReferenceFrame::Response &res) override;
        std::function<kortex_driver::GetCartesianReferenceFrame::Response(const kortex_driver::GetCartesianReferenceFrame::Request&)> GetCartesianReferenceFrameHandler = nullptr;
        virtual bool GetCartesianReferenceFrame(kortex_driver::GetCartesianReferenceFrame::Request  &req, kortex_driver::GetCartesianReferenceFrame::Response &res) override;
        std::function<kortex_driver::ControlConfig_GetControlMode::Response(const kortex_driver::ControlConfig_GetControlMode::Request&)> ControlConfig_GetControlModeHandler = nullptr;
        virtual bool ControlConfig_GetControlMode(kortex_driver::ControlConfig_GetControlMode::Request  &req, kortex_driver::ControlConfig_GetControlMode::Response &res) override;
        std::function<kortex_driver::SetJointSpeedSoftLimits::Response(const kortex_driver::SetJointSpeedSoftLimits::Request&)> SetJointSpeedSoftLimitsHandler = nullptr;
        virtual bool SetJointSpeedSoftLimits(kortex_driver::SetJointSpeedSoftLimits::Request  &req, kortex_driver::SetJointSpeedSoftLimits::Response &res) override;
        std::function<kortex_driver::SetTwistLinearSoftLimit::Response(const kortex_driver::SetTwistLinearSoftLimit::Request&)> SetTwistLinearSoftLimitHandler = nullptr;
        virtual bool SetTwistLinearSoftLimit(kortex_driver::SetTwistLinearSoftLimit::Request  &req, kortex_driver::SetTwistLinearSoftLimit::Response &res) override;
        std::function<kortex_driver::SetTwistAngularSoftLimit::Response(const kortex_driver::SetTwistAngularSoftLimit::Request&)> SetTwistAngularSoftLimitHandler = nullptr;
        virtual bool SetTwistAngularSoftLimit(kortex_driver::SetTwistAngularSoftLimit::Request  &req, kortex_driver::SetTwistAngularSoftLimit::Response &res) override;
        std::function<kortex_driver::SetJointAccelerationSoftLimits::Response(const kortex_driver::SetJointAccelerationSoftLimits::Request&)> SetJointAccelerationSoftLimitsHandler = nullptr;
        virtual bool SetJointAccelerationSoftLimits(kortex_driver::SetJointAccelerationSoftLimits::Request  &req, kortex_driver::SetJointAccelerationSoftLimits::Response &res) override;
        std::function<kortex_driver::GetKinematicHardLimits::Response(const kortex_driver::GetKinematicHardLimits::Request&)> GetKinematicHardLimitsHandler = nullptr;
        virtual bool GetKinematicHardLimits(kortex_driver::GetKinematicHardLimits::Request  &req, kortex_driver::GetKinematicHardLimits::Response &res) override;
        std::function<kortex_driver::GetKinematicSoftLimits::Response(const kortex_driver::GetKinematicSoftLimits::Request&)> GetKinematicSoftLimitsHandler = nullptr;
        virtual bool GetKinematicSoftLimits(kortex_driver::GetKinematicSoftLimits::Request  &req, kortex_driver::GetKinematicSoftLimits::Response &res) override;
        std::function<kortex_driver::GetAllKinematicSoftLimits::Response(const kortex_driver::GetAllKinematicSoftLimits::Request&)> GetAllKinematicSoftLimitsHandler = nullptr;
        virtual bool GetAllKinematicSoftLimits(kortex_driver::GetAllKinematicSoftLimits::Request  &req, kortex_driver::GetAllKinematicSoftLimits::Response &res) override;
        std::function<kortex_driver::SetDesiredLinearTwist::Response(const kortex_driver::SetDesiredLinearTwist::Request&)> SetDesiredLinearTwistHandler = nullptr;
        virtual bool SetDesiredLinearTwist(kortex_driver::SetDesiredLinearTwist::Request  &req, kortex_driver::SetDesiredLinearTwist::Response &res) override;
        std::function<kortex_driver::SetDesiredAngularTwist::Response(const kortex_driver::SetDesiredAngularTwist::Request&)> SetDesiredAngularTwistHandler = nullptr;
        virtual bool SetDesiredAngularTwist(kortex_driver::SetDesiredAngularTwist::Request  &req, kortex_driver::SetDesiredAngularTwist::Response &res) override;
        std::function<kortex_driver::SetDesiredJointSpeeds::Response(const kortex_driver::SetDesiredJointSpeeds::Request&)> SetDesiredJointSpeedsHandler = nullptr;
        virtual bool SetDesiredJointSpeeds(kortex_driver::SetDesiredJointSpeeds::Request  &req, kortex_driver::SetDesiredJointSpeeds::Response &res) override;
        std::function<kortex_driver::GetDesiredSpeeds::Response(const kortex_driver::GetDesiredSpeeds::Request&)> GetDesiredSpeedsHandler = nullptr;
        virtual bool GetDesiredSpeeds(kortex_driver::GetDesiredSpeeds::Request  &req, kortex_driver::GetDesiredSpeeds::Response &res) override;
        std::function<kortex_driver::ResetGravityVector::Response(const kortex_driver::ResetGravityVector::Request&)> ResetGravityVectorHandler = nullptr;
        virtual bool ResetGravityVector(kortex_driver::ResetGravityVector::Request  &req, kortex_driver::ResetGravityVector::Response &res) override;
        std::function<kortex_driver::ResetPayloadInformation::Response(const kortex_driver::ResetPayloadInformation::Request&)> ResetPayloadInformationHandler = nullptr;
        virtual bool ResetPayloadInformation(kortex_driver::ResetPayloadInformation::Request  &req, kortex_driver::ResetPayloadInformation::Response &res) override;
        std::function<kortex_driver::ResetToolConfiguration::Response(const kortex_driver::ResetToolConfiguration::Request&)> ResetToolConfigurationHandler = nullptr;
        virtual bool ResetToolConfiguration(kortex_driver::ResetToolConfiguration::Request  &req, kortex_driver::ResetToolConfiguration::Response &res) override;
        std::function<kortex_driver::ResetJointSpeedSoftLimits::Response(const kortex_driver::ResetJointSpeedSoftLimits::Request&)> ResetJointSpeedSoftLimitsHandler = nullptr;
        virtual bool ResetJointSpeedSoftLimits(kortex_driver::ResetJointSpeedSoftLimits::Request  &req, kortex_driver::ResetJointSpeedSoftLimits::Response &res) override;
        std::function<kortex_driver::ResetTwistLinearSoftLimit::Response(const kortex_driver::ResetTwistLinearSoftLimit::Request&)> ResetTwistLinearSoftLimitHandler = nullptr;
        virtual bool ResetTwistLinearSoftLimit(kortex_driver::ResetTwistLinearSoftLimit::Request  &req, kortex_driver::ResetTwistLinearSoftLimit::Response &res) override;
        std::function<kortex_driver::ResetTwistAngularSoftLimit::Response(const kortex_driver::ResetTwistAngularSoftLimit::Request&)> ResetTwistAngularSoftLimitHandler = nullptr;
        virtual bool ResetTwistAngularSoftLimit(kortex_driver::ResetTwistAngularSoftLimit::Request  &req, kortex_driver::ResetTwistAngularSoftLimit::Response &res) override;
        std::function<kortex_driver::ResetJointAccelerationSoftLimits::Response(const kortex_driver::ResetJointAccelerationSoftLimits::Request&)> ResetJointAccelerationSoftLimitsHandler = nullptr;
        virtual bool ResetJointAccelerationSoftLimits(kortex_driver::ResetJointAccelerationSoftLimits::Request  &req, kortex_driver::ResetJointAccelerationSoftLimits::Response &res) override;
        std::function<kortex_driver::ControlConfig_OnNotificationControlModeTopic::Response(const kortex_driver::ControlConfig_OnNotificationControlModeTopic::Request&)> ControlConfig_OnNotificationControlModeTopicHandler = nullptr;
        virtual bool ControlConfig_OnNotificationControlModeTopic(kortex_driver::ControlConfig_OnNotificationControlModeTopic::Request  &req, kortex_driver::ControlConfig_OnNotificationControlModeTopic::Response &res) override;
        virtual void cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif) override;

};
#endif
