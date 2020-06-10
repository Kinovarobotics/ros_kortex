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
 
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"

int ToRosData(Kinova::Api::ProductConfiguration::CompleteProductConfiguration input, kortex_driver::CompleteProductConfiguration &output)
{
	
	output.kin = input.kin();
	output.model = input.model();
	ToRosData(input.country_code(), output.country_code);
	output.assembly_plant = input.assembly_plant();
	output.model_year = input.model_year();
	output.degree_of_freedom = input.degree_of_freedom();
	output.base_type = input.base_type();
	output.end_effector_type = input.end_effector_type();
	output.vision_module_type = input.vision_module_type();
	output.interface_module_type = input.interface_module_type();
	output.arm_laterality = input.arm_laterality();
	output.wrist_type = input.wrist_type();

	
	
	return 0;
}
int ToRosData(Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType input, kortex_driver::ProductConfigurationEndEffectorType &output)
{
	
	output.end_effector_type = input.end_effector_type();

	
	
	return 0;
}
