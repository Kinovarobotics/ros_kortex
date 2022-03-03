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

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.00538792360024583
pose_target.orientation.x = -0.7074188212758765
pose_target.orientation.y = -0.70675748547511
pose_target.orientation.z = 0.004841312717417838
pose_target.position.x = 0.27555874543377157
pose_target.position.y = 0.0014504323792694802
pose_target.position.z = 1.0090886025521224
group.set_pose_target(pose_target)

plan1 = group.plan()
group.go(wait=True)
rospy.sleep(5)

moveit_commander.roscpp_shutdown()
