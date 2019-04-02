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
from kortex_actuator_driver.srv import *
from kortex_actuator_driver.msg import *
from kortex_device_manager.srv import *
from kortex_device_manager.msg import *

def _function_SetControlLoopParameters():
    rospy.wait_for_service('SetControlLoopParameters')
    
    try:
        function_SetControlLoopParameters = rospy.ServiceProxy('SetControlLoopParameters', SetControlLoopParameters)
        function_SetDeviceID = rospy.ServiceProxy('SetDeviceID', SetDeviceID)
        function_ReadAllDevices = rospy.ServiceProxy('ReadAllDevices', ReadAllDevices)

        req = SetControlLoopParametersRequest()
        reqReadAllDevices = ReadAllDevicesRequest()
        reqSetDeviceID = SetDeviceIDRequest()

        req.input.loop_selection = ControlLoopSelection.MOTOR_VELOCITY;

        actuatorFound = False;
        actuator_id = 0;

        responseReadAllDevices = function_ReadAllDevices(reqReadAllDevices)

        for index, device in enumerate(responseReadAllDevices.output.device_handle):
            if device.device_type is DeviceTypes.BIG_ACTUATOR or device.device_type is DeviceTypes.SMALL_ACTUATOR:
                actuator_id = device.device_identifier
                actuatorFound = True
                reqSetDeviceID.device_id = actuator_id
                function_SetDeviceID(reqSetDeviceID)
                break

        if actuatorFound:
            function_SetControlLoopParameters(req)

    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    _function_SetControlLoopParameters()