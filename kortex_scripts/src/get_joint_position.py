#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('get_moveit_data', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description") 
group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")

print ("Reference frame: %s" % group.get_planning_frame())
print ("End effector: %s" % group.get_end_effector_link())
print ("Current Joint Values:")
print (group.get_current_joint_values())
print ("Robot Groups:")
print (robot.get_group_names())
print ("Current Pose:")
print (group.get_current_pose())
print ("Robot State:")
print (robot.get_current_state())