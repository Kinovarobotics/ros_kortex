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
 
#include "kortex_driver/generated/controlconfig_ros_converter.h"

int ToRosData(Kinova::Api::ControlConfig::GravityVector input, kortex_driver::GravityVector &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::Position input, kortex_driver::ControlConfig_Position &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::PayloadInformation input, kortex_driver::PayloadInformation &output)
{
	
	output.payload_mass = input.payload_mass();
	ToRosData(input.payload_mass_center(), output.payload_mass_center);

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::CartesianTransform input, kortex_driver::CartesianTransform &output)
{
	
	output.x = input.x();
	output.y = input.y();
	output.z = input.z();
	output.theta_x = input.theta_x();
	output.theta_y = input.theta_y();
	output.theta_z = input.theta_z();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::ToolConfiguration input, kortex_driver::ToolConfiguration &output)
{
	
	ToRosData(input.tool_transform(), output.tool_transform);
	output.tool_mass = input.tool_mass();
	ToRosData(input.tool_mass_center(), output.tool_mass_center);

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::ControlConfigurationNotification input, kortex_driver::ControlConfigurationNotification &output)
{
	
	output.event = input.event();
	ToRosData(input.timestamp(), output.timestamp);
	ToRosData(input.user_handle(), output.user_handle);
	ToRosData(input.connection(), output.connection);

	
	
	return 0;
}
int ToRosData(Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input, kortex_driver::CartesianReferenceFrameInfo &output)
{
	
	output.reference_frame = input.reference_frame();

	
	
	return 0;
}
