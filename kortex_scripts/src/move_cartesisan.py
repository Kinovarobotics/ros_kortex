#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('move_group_python', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description")
scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")    
group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
"""
pose: 
  position: 
    x: 0.21870236174029328
    y: -0.07132525403895153
    z: 0.8022677259334249
  orientation: 
    x: -0.00951493622721365
    y: -0.9998928588068303
    z: -0.00018338421950054657
    w: 0.01112219699181503
"""

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.01112219699181503
pose_target.orientation.x = -0.00951493622721365
pose_target.orientation.y = -0.9998928588068303
pose_target.orientation.z = -0.00018338421950054657
pose_target.position.x = 0.21870236174029328
pose_target.position.y = -0.07132525403895153
pose_target.position.z = 0.8022677259334249
group.set_pose_target(pose_target)

plan1 = group.plan()
group.go(wait=True)
rospy.sleep(5)

moveit_commander.roscpp_shutdown()
