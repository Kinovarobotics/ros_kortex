/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
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
 
#include "devicemanager_ros_converter.h"

#include "common_ros_converter.h"


int ToRosData(DeviceHandles input, kortex_device_manager::DeviceHandles &output)
{ 
	output.device_handle.clear();
	for(int i = 0; i < input.device_handle_size(); i++)
	{
		kortex_device_manager::DeviceHandle temp;
		ToRosData(input.device_handle(i), temp);
		output.device_handle.push_back(temp);
	}
	
	return 0;
}
