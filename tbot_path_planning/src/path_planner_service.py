#!/usr/bin/env python3
from email.headerregistry import Group
from genericpath import isfile
from os.path import exists, abspath, realpath
from os import getcwd
from queue import Empty
import sys
from std_msgs.msg import String
from std_srvs.srv import Empty
import rospy
import moveit_commander
import signal
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
import yaml
import time
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, PositionIKRequest
from moveit_msgs.srv import GetPositionIK

from tbot_path_planning.srv import tbotPathPlanMessage, tbotPathPlanMessageResponse

class KortexPathPlanner:
    def my_callback(self, request):
        print("request_to_execute: " + str(request.path_to_file))
        response = tbotPathPlanMessageResponse()
        real_path = realpath(__file__)
        file_path = abspath(real_path+"/../../position_files/"+ str(request.path_to_file))
        if not isfile(file_path):
            print("ERROR 3: file does not exist, try again")
            response.sucess == False
        else:
            self.file_name = file_path
            contents = self.read_from_file(self.file_name)
            response.sucess = self.execute_file(contents["sequence"])

        print("done!")
        if len(sys.argv) > 3:
            print("r/w/a/x")
        return  response # the service Response class, in this case MyCustomServiceMessageResponse

    def __init__(self):
        rospy.init_node('path_planner_service', anonymous=True)
        self.joint_state_topic = ['joint_states:=/my_gen3/joint_states']
        self.my_service = rospy.Service('/tbot_path_planner', tbotPathPlanMessage , self.my_callback) # create the Service called my_service with the defined callback
        moveit_commander.roscpp_initialize(self.joint_state_topic)
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description") 
        self.group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
        self.scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")
        self.pub = rospy.Publisher('/chatter', String, queue_size=10)
        self.on_srv = rospy.ServiceProxy('/my_gen3/mag_gripper/on', Empty)
        self.off_srv = rospy.ServiceProxy('/my_gen3/mag_gripper/off', Empty)
        self.fk_srv = rospy.ServiceProxy('/my_gen3/compute_fk', GetPositionFK)
        self.ik_srv = rospy.ServiceProxy('/my_gen3/compute_ik', GetPositionIK)
        self.rate = rospy.Rate(0.5) # 10hz
        self.responses = {"q1": "r w a x", "primative_functions": "move sleep"}
        self.js_name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.file_name = None

    

    def wait_for_state_update(self, box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_tool_collision_object(self):
        box_name = 'tool'
        box_pose = PoseStamped()
        box_pose.header.frame_id = "mag_gripper_end_effector"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.1

        eef_link = self.group.get_end_effector_link()
        grasping_group = 'eef'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link, box_name, touch_links=touch_links, pose=box_pose, size=(0.1, 0.1, 0.1))
        rospy.loginfo(self.wait_for_state_update(box_name, self.scene, box_is_attached=True, box_is_known=False))

    def remove_tool_colision_object(self):
        self.scene.remove_attached_object(name = 'tool')

    def get_fk(self, joint_state, fk_link, frame_id):
        """
        Do an FK call to with.
        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [fk_link]
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp
    
    def get_ik(self, pose):
        req = PositionIKRequest()
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = self.group.get_planning_frame()
        req.group_name = "arm"
        req.pose_stamped = ps
        req.robot_state = self.group.get_current_state()
        req.avoid_collisions = True
        req.ik_link_name = self.group.get_end_effector_link()
        req.timeout = rospy.Duration(10)
        return self.ik_srv(req)

    def recursive_deepcopy_pose(self, original, copy): #geometry_msgs.msg.Pose()
        if "reference" in original:
            copy = self.recursive_deepcopy_pose(original["reference"], copy)
        if "coordinate_system" in original and original["coordinate_system"] == "joints":
             js = self.get_ik(copy)
             js_list = list(js.solution.joint_state.position)
             for i in range(len(original["position"])):
                js_list[i] += original["position"][i]
             joint_state = JointState()
             joint_state.name = self.js_name
             joint_state.position = js_list[:-1]
             ik = self.get_fk(joint_state, 'mag_gripper_end_effector', 'base_link')
             copy = ik.pose_stamped[0].pose
        else:
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

    def generate_multi_point_plan(self, waypoints, itterations, timeout):
        plans = []
        points = []
        plan = None
        for i in range(itterations):
            fraction = 0
            start_time = time.time()
            while fraction<1.0:
                (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0)  # waypoints to follow # eef_step # jump_threshold 
                # print("fraction: " + str(fraction))
                end_time = time.time()
                if end_time-start_time > timeout:
                    print("Time out: " + str(timeout) + " reached before fraction: " + str(fraction))
                    break
            
            plans.append(plan)
            points.append(len(plans[i].joint_trajectory.points))
        
        shortest_plan =  min(range(len(points)), key=points.__getitem__)
        return plans[shortest_plan]

    def execute_shortest_plan(self, waypoints, itterations):
        if len(waypoints) == 1:
            plan = self.generate_single_point_plan(itterations, waypoints[0], )
            self.group.execute(plan, wait=True)

        elif len(waypoints) > 1:
            plan = self.generate_multi_point_plan(waypoints, itterations, 1)
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
                    # print(type(x["eef"][i]))
                    if x["eef"][i] and type(x["eef"][i]) == bool:
                        print("sending on" + str(self.on_srv()))
                        self.add_tool_collision_object()
                    elif not x["eef"][i] and type(x["eef"][i]) == bool:
                        print("sending off" + str(self.off_srv()))
                        self.remove_tool_colision_object()
                    else:
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
            
        return True        

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
        # print(len(sys.argv))
        if len(sys.argv) > 3:
            while True:
                response = input("r/w/a/x \n") 
                while response not in self.responses["q1"]: # wait for valid response
                    print("ERROR 0: invalid response")
                    response = input("r/w/a/x \n") 

                file_name = input("file name?\n")
                while not exists(file_name) and ("r" in response or "x" in response):
                    print("ERROR 2: file does not exist, try again")
                    file_name = input("file name?\n")
                self.file_name = file_name

                if "r" in response:
                    contents = self.read_from_file(self.file_name)
                    print(contents["sequence"])
                elif "w" in response:
                    print("Not yet implemented: 2")
                elif "x" in response:
                    contents = self.read_from_file(self.file_name)
                    self.execute_file(contents["sequence"])

                else:
                    contents = self.append_to_file(self.file_name)
        else: 
            rospy.spin()

if __name__ == "__main__":
    ex = KortexPathPlanner()
    ex.main()