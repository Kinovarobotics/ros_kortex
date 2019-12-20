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
import threading
from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        rospy.init_node('example_cartesian_poses_with_notifications_python')

        self.HOME_ACTION_IDENTIFIER = 2

        self.pose_1_done = threading.Event()
        self.pose_2_done = threading.Event()
        self.pose_3_done = threading.Event()

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")

        rospy.loginfo("Using robot_name " + self.robot_name)

        # Init the services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)

    def notification_callback(self, notif):
        if notif.action_event == ActionEvent.ACTION_END:
            rospy.loginfo("Action has ended.")
            if notif.handle.identifier == 1001:
                self.pose_1_done.set()
            if notif.handle.identifier == 1002:
                self.pose_2_done.set()
            if notif.handle.identifier == 1003:
                self.pose_3_done.set()
        if notif.action_event == ActionEvent.ACTION_ABORT:
            rospy.logerr("Action was aborted!")
            if notif.handle.identifier == 1001:
                self.pose_1_done.set()
            if notif.handle.identifier == 1002:
                self.pose_2_done.set()
            if notif.handle.identifier == 1003:
                self.pose_3_done.set()

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        # Subscribe to the ActionNotification with the given callback
        rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.notification_callback)

        rospy.sleep(1.0)

    def main(self):
        #*******************************************************************************
        # Make sure to clear the robot's faults else it won't move if it's already in fault
        self.example_clear_faults()
        #*******************************************************************************

        #*******************************************************************************
        # Set the reference frame to "Mixed"
        self.example_set_cartesian_reference_frame()

        #*******************************************************************************
        # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
        self.example_subscribe_to_a_robot_notification()

        #*******************************************************************************
        # Prepare and send pose 1
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.1 # m/s
        my_cartesian_speed.orientation = 15  # deg/s

        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

        my_constrained_pose.target_pose.x = 0.374
        my_constrained_pose.target_pose.y = 0.081
        my_constrained_pose.target_pose.z = 0.450
        my_constrained_pose.target_pose.theta_x = -57.6
        my_constrained_pose.target_pose.theta_y = 91.1
        my_constrained_pose.target_pose.theta_z = 2.3

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        req.input.name = "pose1"
        req.input.handle.identifier = 1001

        rospy.loginfo("Sending pose 1...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose 1")
        else:
            rospy.loginfo("Waiting for pose 1 to finish...")

        self.pose_1_done.wait()

        # Prepare and send pose 2
        req.input.handle.identifier = 1002
        req.input.name = "pose2"

        my_constrained_pose.target_pose.z = 0.3

        req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

        rospy.loginfo("Sending pose 2...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose 2")
        else:
            rospy.loginfo("Waiting for pose 2 to finish...")

        self.pose_2_done.wait()

        # Prepare and send pose 3
        req.input.handle.identifier = 1003
        req.input.name = "pose3"

        my_constrained_pose.target_pose.z = 0.20

        req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

        rospy.loginfo("Sending pose 3...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose 3")
        else:
            rospy.loginfo("Waiting for pose 3 to finish...")

        self.pose_3_done.wait()

        rospy.loginfo("Example finished")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
