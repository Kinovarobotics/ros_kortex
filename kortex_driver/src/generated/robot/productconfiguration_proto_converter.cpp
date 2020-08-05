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
 
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"

int ToProtoData(kortex_driver::CompleteProductConfiguration input, Kinova::Api::ProductConfiguration::CompleteProductConfiguration *output)
{
	
	output->set_kin(input.kin);
	output->set_model((Kinova::Api::ProductConfiguration::ModelId)input.model); 
	ToProtoData(input.country_code, output->mutable_country_code());
	output->set_assembly_plant(input.assembly_plant);
	output->set_model_year(input.model_year);
	output->set_degree_of_freedom(input.degree_of_freedom);
	output->set_base_type((Kinova::Api::ProductConfiguration::BaseType)input.base_type);
	output->set_end_effector_type((Kinova::Api::ProductConfiguration::EndEffectorType)input.end_effector_type);
	output->set_vision_module_type((Kinova::Api::ProductConfiguration::VisionModuleType)input.vision_module_type);
	output->set_interface_module_type((Kinova::Api::ProductConfiguration::InterfaceModuleType)input.interface_module_type);
	output->set_arm_laterality((Kinova::Api::ProductConfiguration::ArmLaterality)input.arm_laterality);
	output->set_wrist_type((Kinova::Api::ProductConfiguration::WristType)input.wrist_type);
	
	return 0;
}
int ToProtoData(kortex_driver::ProductConfigurationEndEffectorType input, Kinova::Api::ProductConfiguration::ProductConfigurationEndEffectorType *output)
{
	
	output->set_end_effector_type((Kinova::Api::ProductConfiguration::EndEffectorType)input.end_effector_type);
	
	return 0;
}
