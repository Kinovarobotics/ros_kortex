#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import json
import signal
import rospy

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('get_moveit_data', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description") 
group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    f.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

name = input("enter name of file, ctrl c to quit \n")

try:
    f = open(name, mode='a', encoding='utf-8')

except IOError:
    print("Error")

while True:
    
    input("press enter to save position, ctrl c to quit \n")
    pos = group.get_current_pose().pose
    pos_dict = {"position":{ "x": pos.position.x,
                            "y": pos.position.y,
                            "z": pos.position.z
                },
                "orientation":{ "x": pos.orientation.x,
                                "y": pos.orientation.y,
                                "z": pos.orientation.z,
                                "w": pos.orientation.w
                },
    }

    f.write(json.dumps(pos_dict) + "\n")
    print("wrote: " + json.dumps(pos_dict) + "\n")