#! /bin/python3


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import moveit_commander
from moveit_msgs.msg import RobotState, Constraints, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('get_ik', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description")
scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")    
arm = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
rospy.wait_for_service('my_gen3/compute_ik')

try:
  moveit_ik = rospy.ServiceProxy('my_gen3/compute_ik', GetPositionIK)
except rospy.ServiceException as e:
  rospy.logerror("Service call failed: %s"%e)




ps = PoseStamped()
ps.pose.position.x = 0.20228798672323686
ps.pose.position.y = -0.0844410319119024
ps.pose.position.z = 0.8287765803854372
ps.pose.orientation.x = 0.004293068374565495
ps.pose.orientation.y = 0.9999850315352882
ps.pose.orientation.z = 0.0033686968829585847
ps.pose.orientation.w = 0.0003976815453087705
ps.header.frame_id = arm.get_planning_frame()

req = PositionIKRequest()
req.group_name = "arm"
req.pose_stamped = ps
req.robot_state = arm.get_current_state()
req.avoid_collisions = False
req.ik_link_name = arm.get_end_effector_link()
req.timeout = rospy.Duration(10)
#req.attempts = 0

resp = moveit_ik(req)
print (resp)
