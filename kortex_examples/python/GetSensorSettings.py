#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
from kortex_vision_config_driver.srv import *
from kortex_vision_config_driver.msg import *
from kortex_device_manager.srv import *
from kortex_device_manager.msg import *

def GetSensorSettings_Client():

    rospy.wait_for_service('ReadAllDevices')

    try:
        function_SetDeviceID = rospy.ServiceProxy('SetDeviceID', SetDeviceID)
        function_ReadAllDevices = rospy.ServiceProxy('ReadAllDevices', ReadAllDevices)
        
        reqReadAllDevices = ReadAllDevicesRequest()
        reqSetDeviceID = SetDeviceIDRequest()

        responseReadAllDevices = function_ReadAllDevices(reqReadAllDevices)
        
        for index, device in enumerate(responseReadAllDevices.output.device_handle):
            if device.device_type is DeviceTypes.VISION:
                device_id = device.device_identifier
                vision = True
                reqSetDeviceID.device_id = device_id
                function_SetDeviceID(reqSetDeviceID)
                break

    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

    if vision is True:
        rospy.wait_for_service('GetSensorSettings')
        try:
            function_GetSensorSettings = rospy.ServiceProxy('GetSensorSettings', GetSensorSettings)

            req = GetSensorSettingsRequest()

            req.input.sensor = Sensor.SENSOR_COLOR;

            response = function_GetSensorSettings(req)

            print(response)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

    else:
        print("Cannot find vision module")


if __name__ == "__main__":
    GetSensorSettings_Client()