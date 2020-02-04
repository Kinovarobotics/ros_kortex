#!/usr/bin/env python

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
# Modified by Alexandre Vannobel to initialize simulated Kinova Gen3 robot in Gazebo 

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty

class ExampleInitializeGazeboRobot(object):
  """home_robot"""
  def __init__(self):
    # Initialize the node
    super(ExampleInitializeGazeboRobot, self).__init__()
    rospy.init_node('init_robot')

    try:
      # Create the MoveItInterface necessary objects
      moveit_commander.roscpp_initialize(sys.argv)
      group_name = "arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)
    except Exception as e:
      rospy.logerr(e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def home_the_robot(self):
    # Home the robot
    self.group.set_named_target("home")
    return self.group.go(wait=True)

def main():
  try:
    # For testing purposes
    try:
        rospy.delete_param("is_initialized")
    except:
        pass

    # Unpause the physics
    # This will let MoveIt finish its initialization
    rospy.loginfo("Unpausing")
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()
    rospy.loginfo("Unpaused")

    example = ExampleInitializeGazeboRobot()
    rospy.loginfo("Created example")
    success = example.is_init_success
    rospy.loginfo("success = {}".format(success))
    if success:
      success &= example.home_the_robot()

  except:
    success = False
  
  # For testing purposes
  rospy.set_param("is_initialized", success)
  if not success:
    rospy.logerr("The Gazebo initialization encountered an error.")
  else:
    rospy.loginfo("The Gazebo initialization executed without fail.")

if __name__ == '__main__':
  main()
