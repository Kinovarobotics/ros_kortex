#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import inspect

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('move_group_python', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description")
scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")    
group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# TO-DO: figure out why the joint move function doenst work in my implementation but works in example
#gripper_max_absolute_pos = gripper_joint.max_bound()
#gripper_min_absolute_pos = gripper_joint.min_bound()
#print(gripper_joint.value())
#print(gripper_joint.move(0.5, True)) # doesnt work for some reason

joints = group.get_current_joint_values()
gripper_joint = robot.get_joint(group.get_active_joints()[0])
joints[0] = 0
group.set_joint_value_target(joints)
plan2 = group.plan()
group.go(wait=True)

moveit_commander.roscpp_shutdown()