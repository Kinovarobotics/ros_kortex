#!/usr/bin/env python3
# from turtle import position
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import json
import signal
import sys

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('move_group_python', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description")
scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")    
arm = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
gripper = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="gripper")

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def deepcopy_pose(original):
    copy = geometry_msgs.msg.Pose()
    copy.position.x = original["position"]["x"]
    copy.position.y = original["position"]["y"]
    copy.position.z = original["position"]["z"]
    copy.orientation.x = original["orientation"]["x"]
    copy.orientation.y = original["orientation"]["y"]
    copy.orientation.z = original["orientation"]["z"]
    copy.orientation.w = original["orientation"]["w"]
    # print(copy)
    return copy

def get_poses_from_file(name):
    num_lines = 0
    try:
         num_lines = sum(1 for line in open(name, mode='r', encoding='utf-8'))
         f = open(name, mode='r', encoding='utf-8')
    except IOError:
        print("Error")
    
    print(str(num_lines) + " poses!")
    pose_list = []
    for i in range(num_lines):
        pose = json.loads(f.readline()) 
        # print(pose)
        pose_list.append(deepcopy_pose(pose))
    return pose_list

def execute_shortest_plan(group, pose_target):
  group.set_pose_target(pose_target)
  plans = []
  points = []

  for i in range(20):
    plans.append(list(group.plan()))
    points.append(len(plans[i][1].joint_trajectory.points))
  
  shortest_plan =  min(range(len(points)), key=points.__getitem__)
  print(points)
  print(shortest_plan)

  group.execute(plans[shortest_plan][1], wait=True)
    

name = input("enter name of file, ctrl c to quit \n")
pose_list = get_poses_from_file(name)
arm.set_goal_tolerance(0.001)
arm.set_planner_id("RRTConnect")
for i in range(10):
    for x in pose_list:
        execute_shortest_plan(arm, x)
moveit_commander.roscpp_shutdown()



