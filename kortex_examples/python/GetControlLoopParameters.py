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

def GetControlLoopParameters_Client():
    rospy.wait_for_service('GetControlLoopParameters')
    
    try:
        function_GetControlLoopParameters = rospy.ServiceProxy('GetControlLoopParameters', GetControlLoopParameters)
        function_SetDeviceID = rospy.ServiceProxy('SetDeviceID', SetDeviceID)
        function_ReadAllDevices = rospy.ServiceProxy('ReadAllDevices', ReadAllDevices)

        reqGetControlLoopParameters = GetControlLoopParametersRequest()
        reqReadAllDevices = ReadAllDevicesRequest()
        reqSetDeviceID = SetDeviceIDRequest()

        reqGetControlLoopParameters.input.loop_selection = ControlLoopSelection.MOTOR_VELOCITY;

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
            responseGetControlLoopParameters = function_GetControlLoopParameters(reqGetControlLoopParameters)

            print("loop_selection = {}".format(responseGetControlLoopParameters.output.loop_selection))
            print("error_saturation = {}".format(responseGetControlLoopParameters.output.error_saturation))
            print("output_saturation = {}\n".format(responseGetControlLoopParameters.output.output_saturation))

            for index, gainA in enumerate(responseGetControlLoopParameters.output.kAz):
                print("kAz[{}] = {}".format(index, gainA))
            
            print("\n")
            
            for index, gainB in enumerate(responseGetControlLoopParameters.output.kBz):
                print("kBz[{}] = {}".format(index, gainB))

    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    GetControlLoopParameters_Client()