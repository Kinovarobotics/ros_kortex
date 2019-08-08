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
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)

    # Create the MoveItInterface necessary objects
    arm_group_name = "arm"
    self.robot = moveit_commander.RobotCommander("robot_description")
    self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
    self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    if self.is_gripper_present:
      gripper_group_name = "gripper"
      self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

    rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    joint_positions[0] = pi/2
    joint_positions[1] = 0
    joint_positions[2] = pi/4
    joint_positions[3] = -pi/4
    joint_positions[4] = 0
    joint_positions[5] = pi/2
    joint_positions[6] = 0.2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      #import pdb; pdb.set_trace()
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint("gripper_finger1_joint")
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_joint.move(relative_position * gripper_max_absolute_pos, True)

def main():
  example = ExampleMoveItTrajectories()

  rospy.loginfo("Press any key to start Named Target Vertical sub example")
  raw_input()
  example.reach_named_position("vertical")

  rospy.loginfo("Press any key to start Reach Joint Angles sub example")
  raw_input()
  example.reach_joint_angles(tolerance=0.01) #rad

  rospy.loginfo("Press any key to start Named Target Home sub example")
  raw_input()
  example.reach_named_position("home")

  rospy.loginfo("Press any key to start Reach Cartesian Pose sub example")
  raw_input()
  actual_pose = example.get_cartesian_pose()
  actual_pose.position.x -= 0.1
  actual_pose.position.y += 0.3
  actual_pose.position.z += 0.3
  example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
  
  if example.is_gripper_present:

    rospy.loginfo("Press any key to start Reach Cartesian Pose With Constraints sub example")
    raw_input()
    
    # Get actual pose
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.y -= 0.3
    
    # Orientation constraint (we want the end effector to stay the same orientation)
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.orientation = actual_pose.orientation
    constraints.orientation_constraints.append(orientation_constraint)

    # Send the goal
    example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
    
    rospy.loginfo("Press any key to open the gripper sub example")
    raw_input()
    example.reach_gripper_position(0)

    rospy.loginfo("Press any key to bring the gripper to half-closed sub example")
    raw_input()
    example.reach_gripper_position(0.5)

if __name__ == '__main__':
  main()
