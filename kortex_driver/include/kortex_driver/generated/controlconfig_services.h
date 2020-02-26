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
 
#ifndef _KORTEX_CONTROLCONFIG_SERVICES_H_
#define _KORTEX_CONTROLCONFIG_SERVICES_H_

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>

#include <ControlConfig.pb.h>
#include <ControlConfigClientRpc.h>
#include "kortex_driver/SetGravityVector.h"
#include "kortex_driver/GetGravityVector.h"
#include "kortex_driver/SetPayloadInformation.h"
#include "kortex_driver/GetPayloadInformation.h"
#include "kortex_driver/SetToolConfiguration.h"
#include "kortex_driver/GetToolConfiguration.h"
#include "kortex_driver/OnNotificationControlConfigurationTopic.h"
#include "kortex_driver/ControlConfigurationNotification.h"
#include "kortex_driver/ControlConfig_Unsubscribe.h"
#include "kortex_driver/SetCartesianReferenceFrame.h"
#include "kortex_driver/GetCartesianReferenceFrame.h"
#include "kortex_driver/ControlConfig_GetControlMode.h"
#include "kortex_driver/SetJointSpeedSoftLimits.h"
#include "kortex_driver/SetTwistLinearSoftLimit.h"
#include "kortex_driver/SetTwistAngularSoftLimit.h"
#include "kortex_driver/SetJointAccelerationSoftLimits.h"
#include "kortex_driver/GetKinematicHardLimits.h"
#include "kortex_driver/GetKinematicSoftLimits.h"
#include "kortex_driver/GetAllKinematicSoftLimits.h"
#include "kortex_driver/SetDesiredLinearTwist.h"
#include "kortex_driver/SetDesiredAngularTwist.h"
#include "kortex_driver/SetDesiredJointSpeeds.h"
#include "kortex_driver/GetDesiredSpeeds.h"
#include "kortex_driver/ResetGravityVector.h"
#include "kortex_driver/ResetPayloadInformation.h"
#include "kortex_driver/ResetToolConfiguration.h"
#include "kortex_driver/ResetJointSpeedSoftLimits.h"
#include "kortex_driver/ResetTwistLinearSoftLimit.h"
#include "kortex_driver/ResetTwistAngularSoftLimit.h"
#include "kortex_driver/ResetJointAccelerationSoftLimits.h"

#include "kortex_driver/KortexError.h"
#include "kortex_driver/SetDeviceID.h"
#include "kortex_driver/SetApiOptions.h"
#include "kortex_driver/ApiOptions.h"

using namespace std;

class ControlConfigServices
{
    public:
        ControlConfigServices(ros::NodeHandle& n, Kinova::Api::ControlConfig::ControlConfigClient* controlconfig, uint32_t device_id, uint32_t timeout_ms);

        bool SetDeviceID(kortex_driver::SetDeviceID::Request  &req, kortex_driver::SetDeviceID::Response &res);
        bool SetApiOptions(kortex_driver::SetApiOptions::Request  &req, kortex_driver::SetApiOptions::Response &res);
        bool SetGravityVector(kortex_driver::SetGravityVector::Request  &req, kortex_driver::SetGravityVector::Response &res);
        bool GetGravityVector(kortex_driver::GetGravityVector::Request  &req, kortex_driver::GetGravityVector::Response &res);
        bool SetPayloadInformation(kortex_driver::SetPayloadInformation::Request  &req, kortex_driver::SetPayloadInformation::Response &res);
        bool GetPayloadInformation(kortex_driver::GetPayloadInformation::Request  &req, kortex_driver::GetPayloadInformation::Response &res);
        bool SetToolConfiguration(kortex_driver::SetToolConfiguration::Request  &req, kortex_driver::SetToolConfiguration::Response &res);
        bool GetToolConfiguration(kortex_driver::GetToolConfiguration::Request  &req, kortex_driver::GetToolConfiguration::Response &res);
        bool OnNotificationControlConfigurationTopic(kortex_driver::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::OnNotificationControlConfigurationTopic::Response &res);
        void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif);
        bool ControlConfig_Unsubscribe(kortex_driver::ControlConfig_Unsubscribe::Request  &req, kortex_driver::ControlConfig_Unsubscribe::Response &res);
        bool SetCartesianReferenceFrame(kortex_driver::SetCartesianReferenceFrame::Request  &req, kortex_driver::SetCartesianReferenceFrame::Response &res);
        bool GetCartesianReferenceFrame(kortex_driver::GetCartesianReferenceFrame::Request  &req, kortex_driver::GetCartesianReferenceFrame::Response &res);
        bool ControlConfig_GetControlMode(kortex_driver::ControlConfig_GetControlMode::Request  &req, kortex_driver::ControlConfig_GetControlMode::Response &res);
        bool SetJointSpeedSoftLimits(kortex_driver::SetJointSpeedSoftLimits::Request  &req, kortex_driver::SetJointSpeedSoftLimits::Response &res);
        bool SetTwistLinearSoftLimit(kortex_driver::SetTwistLinearSoftLimit::Request  &req, kortex_driver::SetTwistLinearSoftLimit::Response &res);
        bool SetTwistAngularSoftLimit(kortex_driver::SetTwistAngularSoftLimit::Request  &req, kortex_driver::SetTwistAngularSoftLimit::Response &res);
        bool SetJointAccelerationSoftLimits(kortex_driver::SetJointAccelerationSoftLimits::Request  &req, kortex_driver::SetJointAccelerationSoftLimits::Response &res);
        bool GetKinematicHardLimits(kortex_driver::GetKinematicHardLimits::Request  &req, kortex_driver::GetKinematicHardLimits::Response &res);
        bool GetKinematicSoftLimits(kortex_driver::GetKinematicSoftLimits::Request  &req, kortex_driver::GetKinematicSoftLimits::Response &res);
        bool GetAllKinematicSoftLimits(kortex_driver::GetAllKinematicSoftLimits::Request  &req, kortex_driver::GetAllKinematicSoftLimits::Response &res);
        bool SetDesiredLinearTwist(kortex_driver::SetDesiredLinearTwist::Request  &req, kortex_driver::SetDesiredLinearTwist::Response &res);
        bool SetDesiredAngularTwist(kortex_driver::SetDesiredAngularTwist::Request  &req, kortex_driver::SetDesiredAngularTwist::Response &res);
        bool SetDesiredJointSpeeds(kortex_driver::SetDesiredJointSpeeds::Request  &req, kortex_driver::SetDesiredJointSpeeds::Response &res);
        bool GetDesiredSpeeds(kortex_driver::GetDesiredSpeeds::Request  &req, kortex_driver::GetDesiredSpeeds::Response &res);
        bool ResetGravityVector(kortex_driver::ResetGravityVector::Request  &req, kortex_driver::ResetGravityVector::Response &res);
        bool ResetPayloadInformation(kortex_driver::ResetPayloadInformation::Request  &req, kortex_driver::ResetPayloadInformation::Response &res);
        bool ResetToolConfiguration(kortex_driver::ResetToolConfiguration::Request  &req, kortex_driver::ResetToolConfiguration::Response &res);
        bool ResetJointSpeedSoftLimits(kortex_driver::ResetJointSpeedSoftLimits::Request  &req, kortex_driver::ResetJointSpeedSoftLimits::Response &res);
        bool ResetTwistLinearSoftLimit(kortex_driver::ResetTwistLinearSoftLimit::Request  &req, kortex_driver::ResetTwistLinearSoftLimit::Response &res);
        bool ResetTwistAngularSoftLimit(kortex_driver::ResetTwistAngularSoftLimit::Request  &req, kortex_driver::ResetTwistAngularSoftLimit::Response &res);
        bool ResetJointAccelerationSoftLimits(kortex_driver::ResetJointAccelerationSoftLimits::Request  &req, kortex_driver::ResetJointAccelerationSoftLimits::Response &res);

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::ControlConfig::ControlConfigClient* m_controlconfig;

        ros::NodeHandle m_n;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_ControlConfigurationTopic;
        bool m_is_activated_ControlConfigurationTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceSetGravityVector;
	ros::ServiceServer m_serviceGetGravityVector;
	ros::ServiceServer m_serviceSetPayloadInformation;
	ros::ServiceServer m_serviceGetPayloadInformation;
	ros::ServiceServer m_serviceSetToolConfiguration;
	ros::ServiceServer m_serviceGetToolConfiguration;
	ros::ServiceServer m_serviceOnNotificationControlConfigurationTopic;
	ros::ServiceServer m_serviceControlConfig_Unsubscribe;
	ros::ServiceServer m_serviceSetCartesianReferenceFrame;
	ros::ServiceServer m_serviceGetCartesianReferenceFrame;
	ros::ServiceServer m_serviceControlConfig_GetControlMode;
	ros::ServiceServer m_serviceSetJointSpeedSoftLimits;
	ros::ServiceServer m_serviceSetTwistLinearSoftLimit;
	ros::ServiceServer m_serviceSetTwistAngularSoftLimit;
	ros::ServiceServer m_serviceSetJointAccelerationSoftLimits;
	ros::ServiceServer m_serviceGetKinematicHardLimits;
	ros::ServiceServer m_serviceGetKinematicSoftLimits;
	ros::ServiceServer m_serviceGetAllKinematicSoftLimits;
	ros::ServiceServer m_serviceSetDesiredLinearTwist;
	ros::ServiceServer m_serviceSetDesiredAngularTwist;
	ros::ServiceServer m_serviceSetDesiredJointSpeeds;
	ros::ServiceServer m_serviceGetDesiredSpeeds;
	ros::ServiceServer m_serviceResetGravityVector;
	ros::ServiceServer m_serviceResetPayloadInformation;
	ros::ServiceServer m_serviceResetToolConfiguration;
	ros::ServiceServer m_serviceResetJointSpeedSoftLimits;
	ros::ServiceServer m_serviceResetTwistLinearSoftLimit;
	ros::ServiceServer m_serviceResetTwistAngularSoftLimit;
	ros::ServiceServer m_serviceResetJointAccelerationSoftLimits;
};
#endif
