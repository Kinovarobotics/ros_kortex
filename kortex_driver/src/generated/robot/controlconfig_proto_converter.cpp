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
 
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"

int ToProtoData(kortex_driver::GravityVector input, Kinova::Api::ControlConfig::GravityVector *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	
	return 0;
}
int ToProtoData(kortex_driver::ControlConfig_Position input, Kinova::Api::ControlConfig::Position *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	
	return 0;
}
int ToProtoData(kortex_driver::PayloadInformation input, Kinova::Api::ControlConfig::PayloadInformation *output)
{
	
	output->set_payload_mass(input.payload_mass); 
	ToProtoData(input.payload_mass_center, output->mutable_payload_mass_center());
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianTransform input, Kinova::Api::ControlConfig::CartesianTransform *output)
{
	
	output->set_x(input.x);
	output->set_y(input.y);
	output->set_z(input.z);
	output->set_theta_x(input.theta_x);
	output->set_theta_y(input.theta_y);
	output->set_theta_z(input.theta_z);
	
	return 0;
}
int ToProtoData(kortex_driver::ToolConfiguration input, Kinova::Api::ControlConfig::ToolConfiguration *output)
{
	 
	ToProtoData(input.tool_transform, output->mutable_tool_transform());
	output->set_tool_mass(input.tool_mass); 
	ToProtoData(input.tool_mass_center, output->mutable_tool_mass_center());
	
	return 0;
}
int ToProtoData(kortex_driver::ControlConfigurationNotification input, Kinova::Api::ControlConfig::ControlConfigurationNotification *output)
{
	
	output->set_event((Kinova::Api::ControlConfig::ControlConfigurationEvent)input.event); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
int ToProtoData(kortex_driver::CartesianReferenceFrameInfo input, Kinova::Api::ControlConfig::CartesianReferenceFrameInfo *output)
{
	
	output->set_reference_frame((Kinova::Api::Common::CartesianReferenceFrame)input.reference_frame);
	
	return 0;
}
int ToProtoData(kortex_driver::TwistLinearSoftLimit input, Kinova::Api::ControlConfig::TwistLinearSoftLimit *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode);
	output->set_twist_linear_soft_limit(input.twist_linear_soft_limit);
	
	return 0;
}
int ToProtoData(kortex_driver::TwistAngularSoftLimit input, Kinova::Api::ControlConfig::TwistAngularSoftLimit *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode);
	output->set_twist_angular_soft_limit(input.twist_angular_soft_limit);
	
	return 0;
}
int ToProtoData(kortex_driver::JointSpeedSoftLimits input, Kinova::Api::ControlConfig::JointSpeedSoftLimits *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode);
	output->clear_joint_speed_soft_limits();
	for(int i = 0; i < input.joint_speed_soft_limits.size(); i++)
	{
		output->add_joint_speed_soft_limits(input.joint_speed_soft_limits[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::JointAccelerationSoftLimits input, Kinova::Api::ControlConfig::JointAccelerationSoftLimits *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode);
	output->clear_joint_acceleration_soft_limits();
	for(int i = 0; i < input.joint_acceleration_soft_limits.size(); i++)
	{
		output->add_joint_acceleration_soft_limits(input.joint_acceleration_soft_limits[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::KinematicLimits input, Kinova::Api::ControlConfig::KinematicLimits *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode);
	output->set_twist_linear(input.twist_linear);
	output->set_twist_angular(input.twist_angular);
	output->clear_joint_speed_limits();
	for(int i = 0; i < input.joint_speed_limits.size(); i++)
	{
		output->add_joint_speed_limits(input.joint_speed_limits[i]);
	}
	output->clear_joint_acceleration_limits();
	for(int i = 0; i < input.joint_acceleration_limits.size(); i++)
	{
		output->add_joint_acceleration_limits(input.joint_acceleration_limits[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::KinematicLimitsList input, Kinova::Api::ControlConfig::KinematicLimitsList *output)
{
	 
	output->clear_kinematic_limits_list();
	for(int i = 0; i < input.kinematic_limits_list.size(); i++)
	{
		ToProtoData(input.kinematic_limits_list[i], output->add_kinematic_limits_list());
	}
	
	return 0;
}
int ToProtoData(kortex_driver::DesiredSpeeds input, Kinova::Api::ControlConfig::DesiredSpeeds *output)
{
	
	output->set_linear(input.linear);
	output->set_angular(input.angular);
	output->clear_joint_speed();
	for(int i = 0; i < input.joint_speed.size(); i++)
	{
		output->add_joint_speed(input.joint_speed[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::LinearTwist input, Kinova::Api::ControlConfig::LinearTwist *output)
{
	
	output->set_linear(input.linear);
	
	return 0;
}
int ToProtoData(kortex_driver::AngularTwist input, Kinova::Api::ControlConfig::AngularTwist *output)
{
	
	output->set_angular(input.angular);
	
	return 0;
}
int ToProtoData(kortex_driver::ControlConfig_JointSpeeds input, Kinova::Api::ControlConfig::JointSpeeds *output)
{
	
	output->clear_joint_speed();
	for(int i = 0; i < input.joint_speed.size(); i++)
	{
		output->add_joint_speed(input.joint_speed[i]);
	}
	
	return 0;
}
int ToProtoData(kortex_driver::ControlConfig_ControlModeInformation input, Kinova::Api::ControlConfig::ControlModeInformation *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode);
	
	return 0;
}
int ToProtoData(kortex_driver::ControlConfig_ControlModeNotification input, Kinova::Api::ControlConfig::ControlModeNotification *output)
{
	
	output->set_control_mode((Kinova::Api::ControlConfig::ControlMode)input.control_mode); 
	ToProtoData(input.timestamp, output->mutable_timestamp()); 
	ToProtoData(input.user_handle, output->mutable_user_handle()); 
	ToProtoData(input.connection, output->mutable_connection());
	
	return 0;
}
