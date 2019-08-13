#!/usr/bin/env python
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

class ExampleActuatorConfiguration:
    def __init__(self):
        rospy.init_node('example_actuator_configuration_python')

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.device_id = rospy.get_param('~device_id', 1)
        rospy.loginfo("Using robot_name " + self.robot_name + " and device_id " + str(self.device_id))

        # Init the services
        read_all_devices_full_name = '/' + self.robot_name + '/device_manager/read_all_devices'
        rospy.wait_for_service(read_all_devices_full_name)
        self.read_all_devices = rospy.ServiceProxy(read_all_devices_full_name, ReadAllDevices)

        set_device_id_full_name = '/' + self.robot_name + '/actuator_config/set_device_id'
        rospy.wait_for_service(set_device_id_full_name)
        self.set_device_id = rospy.ServiceProxy(set_device_id_full_name, SetDeviceID)

        clear_faults_full_name = '/' + self.robot_name + '/actuator_config/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, ActuatorConfig_ClearFaults)

        get_control_loop_parameters_full_name = '/' + self.robot_name + '/actuator_config/get_control_loop_parameters'
        rospy.wait_for_service(get_control_loop_parameters_full_name)
        self.get_control_loop_parameters = rospy.ServiceProxy(get_control_loop_parameters_full_name, GetControlLoopParameters)

    def control_loop_to_string(self, control_loop):
        if control_loop == ControlLoopSelection.RESERVED:
            return "RESERVED"
        elif control_loop == ControlLoopSelection.JOINT_POSITION:
            return "JOINT POSITION"
        elif control_loop == ControlLoopSelection.MOTOR_POSITION:
            return "MOTOR POSITION"
        elif control_loop == ControlLoopSelection.JOINT_VELOCITY:
            return "JOINT VELOCITY"
        elif control_loop == ControlLoopSelection.MOTOR_VELOCITY:
            return "MOTOR VELOCITY"
        elif control_loop == ControlLoopSelection.JOINT_TORQUE:
            return "JOINT TORQUE"
        elif control_loop == ControlLoopSelection.MOTOR_CURRENT:
            return "MOTOR CURRENT"
        else:
            return "UNKNOWN CONTROL LOOP TYPE";    

    def example_find_actuators_and_set_device_id(self):
        rospy.loginfo("-------------------------------")
        rospy.loginfo("Finding actuators...")

        actuator_device_ids = []

        try:
            all_devices = self.read_all_devices()
            for device in all_devices.output.device_handle:
                if device.device_type == DeviceTypes.BIG_ACTUATOR or device.device_type == DeviceTypes.SMALL_ACTUATOR:
                    # Add the device_id to the list if we found an actuator
                    actuator_device_ids.append(device.device_identifier)
                    rospy.loginfo("Found an actuator with device id " + str(device.device_identifier))

        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAllDevices.")

        else:
            rospy.loginfo("-------------------------------")

            # Check if the specified device_id is in the list
            if self.device_id in actuator_device_ids:
                rospy.loginfo("Device id " + str(self.device_id) + " is a valid actuator device id.")
            else:
                raise ValueError("Device id " + str(self.device_id) + " does not correspond to an actuator's device id.")

            # We need to set the device ID of the actuator we want to configure
            req = SetDeviceIDRequest()
            req.device_id = self.device_id
            try:
                self.set_device_id(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call SetDeviceId")

    def example_clear_actuator_faults(self):
        try:
            self.clear_faults()
            rospy.loginfo("Faults were cleared successfully on actuator " + str(self.device_id))
        except rospy.ServiceException:
            rospy.logerr("Failed to call ActuatorConfig_ClearFaults")
  
    def example_get_control_loop_parameters(self):
        try:
            control_loop_parameters_response = self.get_control_loop_parameters()
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetControlLoopParameters")
            return None
        else:
            # The msg file can be found at kortex_driver/msg/generated/actuator_config/ControlLoopParameters.msg
            oss = ""
            control_loop_parameters = control_loop_parameters_response.output
            oss = oss + "\nControl Loop parameters :\n"
            oss = oss + "Loop selection : " + self.control_loop_to_string(control_loop_parameters.loop_selection) + "\n"
            oss = oss + "Error saturation : " + str(control_loop_parameters.error_saturation) + "\n"
            oss = oss + "Output saturation : " + str(control_loop_parameters.output_saturation) + "\n"
            oss = oss + "kAz : ["
            for element in control_loop_parameters.kAz:
                oss = oss + str(element) + " ;"
            oss = oss + "]\n"
            oss = oss + "kBz : ["
            for element in control_loop_parameters.kBz:
                oss = oss + str(element) + " ; "
            oss = oss + "]\n"
            oss = oss + "Error dead band : " + str(control_loop_parameters.error_dead_band) + "\n"
            rospy.loginfo(oss)

            return control_loop_parameters

    def main(self):

        #-------------------------------------------------------------
        # Find actuators and set which one we want to configure
        self.example_find_actuators_and_set_device_id()

        #-------------------------------------------------------------
        # Clear the faults on a specific actuator
        self.example_clear_actuator_faults()
        
        #-------------------------------------------------------------
        # Get the control loop parameters on a specific actuator
        control_loop_parameters = self.example_get_control_loop_parameters()

if __name__ == "__main__":
    ex = ExampleActuatorConfiguration()
    ex.main()
