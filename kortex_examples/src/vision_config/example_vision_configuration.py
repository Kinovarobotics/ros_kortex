#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleVisionConfiguration:
    def __init__(self):
        try:
            rospy.init_node('example_vision_configuration_python')

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the services
            get_intrinsic_parameters_full_name = '/' + self.robot_name + '/vision_config/get_intrinsic_parameters'
            rospy.wait_for_service(get_intrinsic_parameters_full_name, 0.5)
            self.get_intrinsic_parameters = rospy.ServiceProxy(get_intrinsic_parameters_full_name, GetIntrinsicParameters)

            get_extrinsic_parameters_full_name = '/' + self.robot_name + '/vision_config/get_extrinsic_parameters'
            rospy.wait_for_service(get_extrinsic_parameters_full_name, 0.5)
            self.get_extrinsic_parameters = rospy.ServiceProxy(get_extrinsic_parameters_full_name, GetExtrinsicParameters)

            get_sensor_settings_full_name = '/' + self.robot_name + '/vision_config/get_sensor_settings'
            rospy.wait_for_service(get_sensor_settings_full_name, 0.5)
            self.get_sensor_settings = rospy.ServiceProxy(get_sensor_settings_full_name, GetSensorSettings)

            set_sensor_settings_full_name = '/' + self.robot_name + '/vision_config/set_sensor_settings'
            rospy.wait_for_service(set_sensor_settings_full_name, 0.5)
            self.set_sensor_settings = rospy.ServiceProxy(set_sensor_settings_full_name, SetSensorSettings)

            get_option_value_full_name = '/' + self.robot_name + '/vision_config/get_option_value'
            rospy.wait_for_service(get_option_value_full_name, 0.5)
            self.get_option_value = rospy.ServiceProxy(get_option_value_full_name, GetOptionValue)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def sensor_type_enum_to_string(self, enum_value):
        s = ""
        if enum_value == Sensor.SENSOR_COLOR:
            s = "Color"
        elif enum_value == Sensor.SENSOR_DEPTH:
            s = "Depth"
        else:
            s = "Unspecified"
        return s

    def resolution_enum_to_string(self, enum_value):
        s = ""
        if enum_value == Resolution.RESOLUTION_320x240:
            s = "320 x 240"
        elif enum_value == Resolution.RESOLUTION_424x240:
            s = "424 x 240"
        elif enum_value == Resolution.RESOLUTION_480x270:
            s = "480 x 270"
        elif enum_value == Resolution.RESOLUTION_640x480:
            s = "640 x 480"
        elif enum_value == Resolution.RESOLUTION_1280x720:
            s = "1280 x 720"
        elif enum_value == Resolution.RESOLUTION_1920x1080:
            s = "1920 x 1080"
        else:
            s = "Unspecified"
        return s

    def framerate_enum_to_string(self, enum_value):
        s = ""
        if enum_value == FrameRate.FRAMERATE_6_FPS:
            s = "6 FPS"
        elif enum_value == FrameRate.FRAMERATE_15_FPS:
            s = "15 FPS"
        elif enum_value == FrameRate.FRAMERATE_30_FPS:
            s = "30 FPS"
        else:
            s = "Unspecified"
        return s

    def bitrate_enum_to_string(self, enum_value):
        s = ""
        if enum_value == BitRate.BITRATE_10_MBPS:
            s = "10 MBPS"  
        elif enum_value == BitRate.BITRATE_15_MBPS:
            s = "15 MBPS"
        elif enum_value == BitRate.BITRATE_20_MBPS:
            s = "20 MBPS"
        elif enum_value == BitRate.BITRATE_25_MBPS:
            s = "25 MBPS"
        else:
            s = "Unspecified"
        return s

    def example_get_intrinsic_parameters(self):
        # Call the service to get the params
        req = GetIntrinsicParametersRequest()
        req.input.sensor = Sensor.SENSOR_COLOR # Change to SENSOR_DEPTH for the depth sensor
        try:
            res = self.get_intrinsic_parameters(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetIntrinsicParameters")
            return False
        else:
            # Print them
            # The message description can be seen at msg/generated/vision_config/IntrinsicParameters.msg
            intrinsic_parameters = res.output
            out = ""
            out += "\n----------------------------------\n"
            out += "Intrinsic parameters are :\n"
            out += "Focal length in x is : " + str(intrinsic_parameters.focal_length_x) + "\n"
            out += "Focal length in y is : " + str(intrinsic_parameters.focal_length_y) + "\n"
            out += "Principal point in x is : " + str(intrinsic_parameters.principal_point_x) + "\n"
            out += "Principal point in y is : " + str(intrinsic_parameters.principal_point_y) + "\n"
            out += "Distortion coefficients are : ["
            out += "k1 = " + str(intrinsic_parameters.distortion_coeffs.k1) + "; "
            out += "k2 = " + str(intrinsic_parameters.distortion_coeffs.k2) + "; "
            out += "k3 = " + str(intrinsic_parameters.distortion_coeffs.k3) + "; "
            out += "p1 = " + str(intrinsic_parameters.distortion_coeffs.p1) + "; "
            out += "p2 = " + str(intrinsic_parameters.distortion_coeffs.p2) + "]" + "\n"

            # The Sensor enum can be seen at msg/generated/vision_config/Sensor.msg
            out += "Sensor type is : " + self.sensor_type_enum_to_string(intrinsic_parameters.sensor) + "\n"

            # The Resolution enum can be seen at msg/generated/vision_config/Resolution.msg
            out += "Resolution is : " + self.resolution_enum_to_string(intrinsic_parameters.resolution) + "\n" 
            out += "----------------------------------"

            rospy.loginfo(out)

            return True

    def example_get_extrinsic_parameters(self):
        # Call the service 
        try:
            res = self.get_extrinsic_parameters()
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetExtrinsicParameters")
            return False
        else:
            # Print the result
            # The message description can be seen at msg/generated/vision_config/ExtrinsicParameters.msg
            extrinsic_parameters = res.output
            out = "\n----------------------------------\n"
            out += "Extrinsic parameters are :\n"
            out += "Rotation parameters matrix is : " + "\n"
            out += "|  " + str(extrinsic_parameters.rotation.row1.column1) + "  ;  " + str(extrinsic_parameters.rotation.row1.column2) + "  ;  " + str(extrinsic_parameters.rotation.row1.column3) + "  |" + "\n"
            out += "|  " + str(extrinsic_parameters.rotation.row2.column1) + "  ;  " + str(extrinsic_parameters.rotation.row2.column2) + "  ;  " + str(extrinsic_parameters.rotation.row2.column3) + "  |" + "\n"
            out += "|  " + str(extrinsic_parameters.rotation.row3.column1) + "  ;  " + str(extrinsic_parameters.rotation.row3.column2) + "  ;  " + str(extrinsic_parameters.rotation.row3.column3) + "  |" + "\n"

            out += "Translation parameters are : "
            out += "[ x = " + str(extrinsic_parameters.translation.t_x)
            out += " ; y = " + str(extrinsic_parameters.translation.t_y)
            out += " ; z = " + str(extrinsic_parameters.translation.t_z) + " ]"
            out += "\n" + "---------------------------------" + "\n"

            rospy.loginfo(out)
            return True

    def example_get_sensor_settings(self):
        # Call the service
        req = GetSensorSettingsRequest()
        req.input.sensor = Sensor.SENSOR_COLOR
        try:
            res = self.get_sensor_settings(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetSensorSettings")
            return False
        else:
            # Print the result
            sensor_settings = res.output
            out = "\n----------------------------------\n"
            out += "Get sensor settings : " "\n"
            out += "Bit rate : " + self.bitrate_enum_to_string(sensor_settings.bit_rate) + "\n"
            out += "Frame rate : " + self.framerate_enum_to_string(sensor_settings.frame_rate) + "\n" 
            out += "Resolution : " + self.resolution_enum_to_string(sensor_settings.resolution) + "\n" 
            out += "---------------------------------"

            rospy.loginfo(out)

            return True

    def example_change_the_resolution(self):
        rospy.loginfo("Changing the resolution...")
        req = SetSensorSettingsRequest()
        # Set the resolution to be 640 x 480 on the color sensor
        # You have to specify all parameters else the call will return an INVALID_PARAM error
        req.input.sensor = Sensor.SENSOR_COLOR
        req.input.resolution = Resolution.RESOLUTION_1280x720
        req.input.bit_rate = BitRate.BITRATE_10_MBPS
        req.input.frame_rate = FrameRate.FRAMERATE_30_FPS
        try:
            self.set_sensor_settings(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetSensorSettings and change the resolution")
            return False
        else:
            rospy.loginfo("Resolution changed successfully")
            return True

    def example_get_sensor_option_value(self):
        req = GetOptionValueRequest()

        # The only supported options for now are:
        # For Color sensor : 
        # OPTION_BRIGHTNESS, OPTION_CONTRAST, OPTION_SATURATION
        # For Depth sensor : 
        # OPTION_EXPOSURE, OPTION_GAIN, OPTION_ENABLE_AUTO_EXPOSURE, OPTION_VISUAL_PRESET, OPTION_FRAMES_QUEUE_SIZE, 
        # OPTION_ERROR_POLLING_ENABLE, OPTION_OUTPUT_TRIGGER_ENABLED, OPTION_DEPTH_UNITS, 
        # OPTION_STEREO_BASELINE (read-only) 
        # Trying to call an unsupported option in this service (or in the SetOptionValue service) will generate an error 
        
        # Get the actual value the color sensor's contrast 
        req.input.sensor = Sensor.SENSOR_COLOR
        req.input.option = Option.OPTION_CONTRAST

        try:
            res = self.get_option_value(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetOptionValue")
            return False
        else:
            option_value = res.output
            out = "\n---------------------------------\n"
            out += "Get Option value :\n" 
            out += "Option value is : \n"
            out += "For sensor : " + self.sensor_type_enum_to_string(option_value.sensor) + "\n"
            # You can see the Option enum at msg/generated/vision_config/Option.msg
            out += "For option : " + str(option_value.option) + "\n"
            out += "The value is : " + str(option_value.value) + "\n"
            out += "---------------------------------\n"

            rospy.loginfo(out)

            return True
            
    def main(self):

         # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/vision_configuration_python")
        except:
            pass

        if success:
            #-------------------------------------------------------------
            # Get the intrinsic parameters for a given sensor
            success &= self.example_get_intrinsic_parameters()

            #-------------------------------------------------------------
            # Get the extrinsic parameters for a given sensor
            success &= self.example_get_extrinsic_parameters()

            #-------------------------------------------------------------
            # Get the sensor settings for a given sensor
            success &= self.example_get_sensor_settings()

            #-------------------------------------------------------------
            # Set the extrinsic parameters for a given sensor
            success &= self.example_change_the_resolution()

            #-------------------------------------------------------------
            # Get an option value
            success &= self.example_get_sensor_option_value()

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/vision_configuration_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleVisionConfiguration()
    ex.main()
