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
 
#include "kortex_driver/generated/controlconfig_proto_converter.h"

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
