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
from kortex_driver.srv import *
from kortex_driver.msg import *

def PlayCartesian_client():
    
    rospy.wait_for_service('PlayCartesianTrajectory')
    rospy.wait_for_service('RefreshFeedback')
    
    try:
        function_PlayCartesianTrajectory = rospy.ServiceProxy('PlayCartesianTrajectory', PlayCartesianTrajectory)
        function_RefreshFeedback = rospy.ServiceProxy('RefreshFeedback', RefreshFeedback)
        
        current_feedback = function_RefreshFeedback()
        request = PlayCartesianTrajectoryRequest()

        current_x = current_feedback.output.base.tool_pose_x
        current_y = current_feedback.output.base.tool_pose_y
        current_z = current_feedback.output.base.tool_pose_z

        current_theta_x = current_feedback.output.base.tool_pose_theta_x
        current_theta_y = current_feedback.output.base.tool_pose_theta_y
        current_theta_z = current_feedback.output.base.tool_pose_theta_z

        #Creating our next target (a Cartesian pose)
        request.input.target_pose.x = current_x
        request.input.target_pose.y = current_y
        request.input.target_pose.z = current_z + 0.1

        request.input.target_pose.theta_x = current_theta_x
        request.input.target_pose.theta_y = current_theta_y
        request.input.target_pose.theta_z = current_theta_z

        poseSpeed = CartesianSpeed()
        poseSpeed.translation = 0.1
        poseSpeed.orientation = 15

        request.input.constraint.oneof_type.speed.append(poseSpeed)

        function_PlayCartesianTrajectory(request)
        
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    PlayCartesian_client()