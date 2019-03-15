#!/usr/bin/env python

import sys
import rospy
import time
from kortex_vision_config_driver.srv import *
from kortex_vision_config_driver.msg import *

def GetSensorSettings_Client():
    rospy.wait_for_service('GetSensorSettings')
    try:
        function_GetSensorSettings = rospy.ServiceProxy('GetSensorSettings', GetSensorSettings)

        req = GetSensorSettingsRequest()

        req.input.sensor = Sensor.SENSOR_COLOR;

        response = function_GetSensorSettings(req)

        print(response)

        

    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    GetSensorSettings_Client()