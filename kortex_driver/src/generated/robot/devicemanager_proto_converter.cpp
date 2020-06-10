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
 
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"

int ToProtoData(kortex_driver::DeviceHandles input, Kinova::Api::DeviceManager::DeviceHandles *output)
{
	 
	output->clear_device_handle();
	for(int i = 0; i < input.device_handle.size(); i++)
	{
		ToProtoData(input.device_handle[i], output->add_device_handle());
	}
	
	return 0;
}
