#!/usr/bin/env python3
from os.path import exists
from asyncore import read
from shutil import move
import sys
from unittest import loader
import rospy
import moveit_commander
import signal
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import yaml
import time

class KortexPathPlanner:

    def __init__(self):
        rospy.init_node('kortex_path_planner', anonymous=True)
        self.joint_state_topic = ['joint_states:=/my_gen3/joint_states']
        moveit_commander.roscpp_initialize(self.joint_state_topic)
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description") 
        self.group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
        self.responses = {"q1": "r w a x"}

    def deepcopy_pose_list(self, original):
        copy_list = []
        for x in original:
            copy = geometry_msgs.msg.Pose()
            copy.position.x = x["position"]["x"] + x["reference"]["position"]["x"]
            copy.position.y = x["position"]["y"] + x["reference"]["position"]["y"]
            copy.position.z = x["position"]["z"] + x["reference"]["position"]["z"]
            copy.orientation.x = x["orientation"]["x"] + x["reference"]["orientation"]["x"]
            copy.orientation.y = x["orientation"]["y"] + x["reference"]["orientation"]["y"]
            copy.orientation.z = x["orientation"]["z"] + x["reference"]["orientation"]["z"]
            copy.orientation.w = x["orientation"]["w"] + x["reference"]["orientation"]["w"]
            copy_list.append(copy)
        return copy_list

    def execute_shortest_plan(self, group, pose_target):
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
            
    def read_from_file(self, file_name):
        try:
            with open(file_name, 'r') as file:
                return yaml.load(file, Loader=yaml.FullLoader)
        except:
            print("ERROR 1: File " + file_name + " cannot be found, opened, or read. Try again")
            return "Error 1"

    def write_to_file(self, data, file_name):
        with open(file_name, 'w') as file:
            documents = yaml.dump(data, file)

    def execute_file(self, file_name):
        with open(file_name, 'r') as file:
            contents = yaml.load(file, Loader=yaml.FullLoader)
            total_actions = len(contents["sequence"])

            for x in contents["sequence"]:
                if "move" in iter(x):
                    waypoints = self.deepcopy_pose_list(x["move"])
                    print(waypoints)
                    for y in waypoints:
                        self.execute_shortest_plan(self.group, y)

                elif "sleep" in iter(x):
                    print("sleeping: " + str(x["sleep"]))
                    time.sleep(x["sleep"])
                    print("woke up")

    def append_to_file(self, data, file_name):
        # with open(file_name, 'a') as file:
        #     documents = yaml.dump(data, file)
        print("not yet implimented")
    
    def main(self):
        def signal_handler(sig, frame):
            print('\nYou pressed Ctrl+C!')
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        self.group.set_goal_tolerance(0.0005)
        self.group.set_planner_id("RRTConnect")

        while True:
            response = input("r/w/a/x \n") 
            while response not in self.responses["q1"]: # wait for valid response
                print("ERROR 0: invalid response")
                response = input("r/w/a/x \n") 

            file_name = input("file name?\n")
            while not exists(file_name) and ("r" in response or "x" in response):
                print("ERROR 2: file does not exist, try again")
                file_name = input("file name?\n")
                
            if "r" in response:
                contents = self.read_from_file(file_name)
                print(contents["sequence"])
            elif "w" in response:
                print("Not yet implemented: 2")
            elif "x" in response:
                self.execute_file(file_name)

            else:
                contents = self.append_to_file(file_name)

if __name__ == "__main__":
    ex = KortexPathPlanner()
    ex.main()