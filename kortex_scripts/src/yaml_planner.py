#!/usr/bin/env python3
from os.path import exists
from asyncore import read
import sys
from unittest import loader
import rospy
import moveit_commander
import signal
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import yaml

class KortexPathPlanner:

    def __init__(self):
        rospy.init_node('kortex_path_planner', anonymous=True)
        self.joint_state_topic = ['joint_states:=/my_gen3/joint_states']
        moveit_commander.roscpp_initialize(self.joint_state_topic)
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description") 
        self.group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
        self.responses = {"q1": "r w a x", "q2": "sequence point none relitive s p n r", "q3": "y n"} 

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

    def append_to_file(self, data, file_name):
        # with open(file_name, 'a') as file:
        #     documents = yaml.dump(data, file)
        print("not yet implimented")

    def deepcopy_pose(original):
        copy = geometry_msgs.msg.Pose()
        copy.position.x = original["position"]["x"]
        copy.position.y = original["position"]["y"]
        copy.position.z = original["position"]["z"]
        copy.orientation.x = original["orientation"]["x"]
        copy.orientation.y = original["orientation"]["y"]
        copy.orientation.z = original["orientation"]["z"]
        copy.orientation.w = original["orientation"]["w"]
        return copy

    def get_gripper_pose_dict(self):
        response = input("gripper sequence/point/<empty> \n")
        while response not in self.responses["q2"]:
            print("ERROR 4: invalid response")
            response = input("sequence/point\n")

        pos_dict = {"states":{"active": False,
                            "leading": False,
                            "num_poses": 0},
                    "positions": None}
                    
        if not response:
            return pos_dict
        else:

            position = input("integer/<empty>\n")
            counter = 0
            nums = []
            while position:
                try:
                    nums.append(int(position))  
                except:
                    print("ERROR 5: invalid response")
                    counter -= 1
                position = input("integer/<empty>\n")
                counter += 1
            
            pos_dict["states"]["active"] = True
            pos_dict["states"]["num_poses"] = counter
            pos_dict.update({"positions": nums})

        response = input("leading? y/n \n")
        while response not in self.responses["q3"]:
            print("ERROR 5: invalid response")
            response = input("leading? y/n \n")
        if "y" in response:
            pos_dict["states"]["leading"] = True
        
        return pos_dict

    def get_arm_pose_dict(self, gripper):
        pos = self.group.get_current_pose().pose
        pos_dict = {"position":{ "x": pos.position.x,
                                "y": pos.position.y,
                                "z": pos.position.z
                                },
                    "orientation":{ "x": pos.orientation.x,
                                    "y": pos.orientation.y,
                                    "z": pos.orientation.z,
                                    "w": pos.orientation.w
                                },
                    "gripper": gripper
                    }
        return pos_dict
    
    def generate_poi(self):
        return {"states": {"active": True,"sequence": 0}, "arm": None}

    def get_poi(self):
        response = input("arm sequence/point/relitive/<empty>\n")
        while response not in self.responses["q2"]:
            print("ERROR 3: invalid response")
            response = input("arm sequence/point/relitive/<empty\n")

        poi = self.generate_poi()

        if response:
            if "p" in response:
                arm = self.get_arm_pose_dict(gripper = self.get_gripper_pose_dict())
                poi.update({"arm": arm})
                return [poi]

            elif "s" in response:
                pose_list = []   
                while not input("<empty>-> save pose | <s>-> save and quit\n"):
                    pose_list.append(self.get_arm_pose_dict(gripper = self.get_gripper_pose_dict()))
                    poi["states"]["sequence"] += 1
                poi.update({"arm": pose_list}) 
                return [poi]

            elif "r" in response:
                print("not yet implemented")
        
        else:
            poi["states"]["active"] = False
            arm = self.get_arm_pose_dict(gripper = self.get_gripper_pose_dict())
            poi.update({"arm": arm})
            return [poi]
            
    def main(self):
        def signal_handler(sig, frame):
            print('\nYou pressed Ctrl+C!')
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        while True:
            response = input("r/w/a/x \n") 
            while response not in self.responses["q1"]: # wait for valid response
                print("ERROR 0: invalid response")
                response = input("r/w/a/x \n") 

            file_name = input("file name?\n")
            while not exists(file_name) and "r" in response:
                print("ERROR 2: file does not exist, try again")
                file_name = input("file name?\n")
                
            if "r" in response:
                contents = self.read_from_file(file_name)
                print(contents)
            elif "w" in response:
                contents = self.write_to_file(self.get_poi(), file_name)

            else:
                contents = self.append_to_file(file_name)


if __name__ == "__main__":
    ex = KortexPathPlanner()
    ex.main()