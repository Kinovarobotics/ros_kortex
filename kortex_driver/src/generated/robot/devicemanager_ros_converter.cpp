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
 
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"

int ToRosData(Kinova::Api::DeviceManager::DeviceHandles input, kortex_driver::DeviceHandles &output)
{
	
	output.device_handle.clear();
	for(int i = 0; i < input.device_handle_size(); i++)
	{
		decltype(output.device_handle)::value_type temp;
		ToRosData(input.device_handle(i), temp);
		output.device_handle.push_back(temp);
	}

	
	
	return 0;
}
