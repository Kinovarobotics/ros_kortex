#!/usr/bin/env python3
from copy import copy
from os.path import exists
import sys
from std_msgs.msg import String
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
        self.pub = rospy.Publisher('/chatter', String, queue_size=10)
        self.rate = rospy.Rate(0.5) # 10hz
        self.responses = {"q1": "r w a x", "primative_functions": "move sleep"}

    def recursive_deepcopy_pose(self, original, copy): #geometry_msgs.msg.Pose()
        if "reference" in original:
            self.recursive_deepcopy_pose(original["reference"], copy)
        copy.position.x += original["position"]["x"]
        copy.position.y += original["position"]["y"]
        copy.position.z += original["position"]["z"]
        copy.orientation.x += original["orientation"]["x"]
        copy.orientation.y += original["orientation"]["y"]
        copy.orientation.z += original["orientation"]["z"]
        copy.orientation.w += original["orientation"]["w"]
        return copy

    def deepcopy_pose_list(self, original):
        waypoints = []
        for x in original:
            waypoints.append(self.recursive_deepcopy_pose(x, geometry_msgs.msg.Pose()))
        return waypoints

    def generate_single_point_plan(self, itterations, pose_target):
        self.group.set_pose_target(pose_target)
        plans = []
        points = []
        for i in range(itterations):
            plans.append(list(self.group.plan()))
            points.append(len(plans[i][1].joint_trajectory.points))

        shortest_plan =  min(range(len(points)), key=points.__getitem__)
        # print(points)
        # print(shortest_plan)
        return plans[shortest_plan][1]

    def generate_multi_point_plan(self, waypoints):
        fraction = 0
        while fraction<1.0:
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0)  # waypoints to follow # eef_step # jump_threshold 
        #     print("fraction: " + str(fraction))
        # print("fraction: " + str(fraction))
        return plan

    def execute_shortest_plan(self, waypoints, itterations):
        if len(waypoints) == 1:
            plan = self.generate_single_point_plan(itterations, waypoints[0])
            self.group.execute(plan, wait=True)

        elif len(waypoints) > 1:
            plan = self.generate_multi_point_plan(waypoints)
            self.group.execute(plan, wait=True)
     
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

    def primative_functions(self, contents):
        for x in contents:
            if "move" in iter(x):
                print("moving")
                waypoints = self.deepcopy_pose_list(x["move"])
                # print(waypoints)
                self.execute_shortest_plan(waypoints, 20)

            elif "sleep" in iter(x):
                print("sleeping: " + str(x["sleep"]))
                time.sleep(x["sleep"])
                print("woke up")

            elif "eef" in iter(x):
                print("eef\n" + str(x))
                for i in range(len(x["eef"])):
                    self.rate.sleep()
                    # print(str(x["eef"][i]))
                    self.pub.publish(str(x["eef"][i]))
                    
    def execute_file(self, sequence):
        for i in range(len(sequence)):

            if "do" in iter(sequence[i]):
                for k in range(sequence[i]["do"]["count"]):
                    print("preforming loop: " + str(k+1) + "/" + str(sequence[i]["do"]["count"]))
                    self.primative_functions(sequence[i]["do"]["loop"])
            
            elif "move" in iter(sequence[i]) or "sleep" in iter(sequence[i]) or "eef" in iter(sequence[i]):
                self.primative_functions([sequence[i]])

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
                contents = self.read_from_file(file_name)
                self.execute_file(contents["sequence"])

            else:
                contents = self.append_to_file(file_name)

if __name__ == "__main__":
    ex = KortexPathPlanner()
    ex.main()