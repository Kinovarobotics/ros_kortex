#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to tune the simulated controller's PID for Gazebo and the Gen3 robot

# Usage : 
# rosrun kortex_examples simulation_pid_tuning.py _inertia:=low _actuator:=3 _sleep_time:=2
# inertia : low or high
# actuator : 1 to max number (7 for Gen3)
# sleep_time : time to sleep between trajectories (in seconds)

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

high_inertia = [[[0, 1.57, 0, 0, 0, 0, 0],[0, 1.57, 0, 0, 0, 0, 0],[0, 1.57, 1.57, 0, 0, 0, 0],[0, 0, 0, 1.57, 0, 0, 0],[0, 0.78, 0, 0.78, 0, 0, 0],[0, 0.78, 0, 0.78, 0, 0.25, 0],[0, 0.78, 0, 0.78, 0, 0, 0]], \
                [[0.25, 1.57, 0, 0, 0, 0, 0],[0, 1.25, 0, 0, 0, 0, 0],[0, 1.57, 1.25, 0, 0, 0, 0],[0, 0, 0, 1.25, 0, 0, 0],[0, 0.78, 0, 0.78, 0.25, 0, 0],[0, 0.78, 0, 0.78, 0, 0, 0],[0, 0.78, 0, 0.78, 0, 0, 0.25]]]

low_inertia = [[[0, 0, 0, 0, 0, 0, 0],[0, 1.57, 1.57, 1.57, 0, 1.57, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 1.57, 0, 1.57, 0],[0, 0, 0, 0, 0, 0, 0],[0, 0.78, 0, 0.78, 0, 1.57, 0],[0, 0, 0, 0, 0, 0, 0]], \
              [[0.25, 0, 0, 0, 0, 0, 0], [0, 1.25, 1.57, 1.57, 0, 1.57, 0], [0, 0, 0.25, 0, 0, 0, 0], [0, 0, 0, 1.25, 0, 1.57, 0], [0, 0, 0, 0, 0.25, 0, 0],[0, 0.78, 0, 0.78, 0, 1.25, 0],[0, 0, 0, 0, 0, 0, 0.25]]]

class MoveItPIDTuning(object):
    """MoveItPIDTuning"""
    def __init__(self):

        # Initialize the node
        super(MoveItPIDTuning, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('MoveItPIDTuning_node')

        self.actuator = rospy.get_param("~actuator")
        if self.actuator < 1 or self.actuator > 7:
            rospy.logerr("Actuator must be between 1 and 7 inclusively for gen3")
            exit(0)
        self.inertia = rospy.get_param("~inertia")
        rospy.loginfo("---{}---".format(self.inertia))
        if self.inertia != "low" and self.inertia != "high":
            rospy.logerr("Inertia must be low or high")
            exit(0)
        self.sleep_time = rospy.get_param("~sleep_time")

        # Create the MoveItInterface necessary objects
        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)

    def reach_joint_angles(self, using_one):
        arm_group = self.arm_group

        # Set the joint target configuration
        joint_target = []
        if self.inertia == "high":
            if using_one is True:
                joint_target = high_inertia[0][self.actuator-1]
            else:
                joint_target = high_inertia[1][self.actuator-1]
        if self.inertia == "low":
            if using_one is True:
                joint_target = low_inertia[0][self.actuator-1]
            else:
                joint_target = low_inertia[1][self.actuator-1]
        
        arm_group.set_joint_value_target(joint_target)
        
        # Plan and execute in one command
        arm_group.go(wait=True)

def main():
    tuner = MoveItPIDTuning()
    rospy.loginfo("Going to first position, then will wait 10 secs before starting")
    tuner.reach_joint_angles(True)
    rospy.sleep(10)
    while(True):
        tuner.reach_joint_angles(True)
        rospy.sleep(tuner.sleep_time)
        tuner.reach_joint_angles(False)
        rospy.sleep(tuner.sleep_time)

if __name__ == '__main__':
    main()
