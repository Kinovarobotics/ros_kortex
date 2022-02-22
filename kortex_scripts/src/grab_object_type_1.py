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
arm = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
gripper = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="gripper")

pose_target_1 = geometry_msgs.msg.Pose()
pose_target_1.position.x = 0.2501254246464173
pose_target_1.position.y = 0.001303065168318192
pose_target_1.position.z = 0.40105974497820623
pose_target_1.orientation.x = -0.7068807873212782
pose_target_1.orientation.y = -0.7072424830115577
pose_target_1.orientation.z = 0.007774958905352527
pose_target_1.orientation.w = 0.008195898591209215
pose_target_2 = copy.deepcopy(pose_target_1)
pose_target_2.position.z = 0.1
pose_target_3 = copy.deepcopy(pose_target_1)
pose_target_3.position.x = -0.2501254246464173
pose_target_4 = copy.deepcopy(pose_target_3)
pose_target_4.position.z = 0.1
pose_list = [pose_target_1, pose_target_2, pose_target_3, pose_target_4]
joints = gripper.get_current_joint_values()

while True:
    arm.set_pose_target(pose_list[0])
    arm.plan()
    arm.go(wait=True)
    arm.set_pose_target(pose_list[1])
    arm.plan()
    arm.go(wait=True)
    joints[0] = 0.8
    gripper.set_joint_value_target(joints)
    gripper.plan()
    gripper.go(wait=True)
    arm.set_pose_target(pose_list[2])
    arm.plan()
    arm.go(wait=True)
    arm.set_pose_target(pose_list[3])
    arm.plan()
    arm.go(wait=True)
    joints[0] = 0
    gripper.set_joint_value_target(joints)
    gripper.plan()
    gripper.go(wait=True)
    arm.set_pose_target(pose_list[2])
    arm.plan()
    arm.go(wait=True)
    arm.set_pose_target(pose_list[3])
    arm.plan()
    arm.go(wait=True)
    joints[0] = 0.8
    gripper.set_joint_value_target(joints)
    gripper.plan()
    gripper.go(wait=True)
    arm.set_pose_target(pose_list[2])
    arm.plan()
    arm.go(wait=True)
    arm.set_pose_target(pose_list[0])
    arm.plan()
    arm.go(wait=True)
    arm.set_pose_target(pose_list[1])
    arm.plan()
    arm.go(wait=True)
    joints[0] = 0
    gripper.set_joint_value_target(joints)
    gripper.plan()
    gripper.go(wait=True)



rospy.sleep(5)

moveit_commander.roscpp_shutdown()
