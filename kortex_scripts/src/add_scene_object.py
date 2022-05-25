#!/usr/bin/env python3
from json import tool
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject

rospy.init_node('scene_object_spawner', anonymous=True)
joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description") 
group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")

def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
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


# ROS answers suggested adding a sleep here to allow the PlanningSceneInterface
# to initialize properly; we have noticed that collision objects aren't
# always received without it.
rospy.sleep(2)

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.5
p.pose.position.y = 0.0
p.pose.position.z = 0.0
scene.add_box("table", p, (1, 1, 0.01))
rospy.loginfo(wait_for_state_update("table", scene, box_is_attached=False, box_is_known=False))

box_pose = PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.position.x = 0.26
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.05
scene.add_box("tool", box_pose, (0.1, 0.1, 0.1))
rospy.loginfo(wait_for_state_update("tool", scene, box_is_attached=False, box_is_known=False))

# eef_link = group.get_end_effector_link()
# grasping_group = 'eef'
# touch_links = robot.get_link_names(group=grasping_group)
# scene.attach_box(eef_link, box_name, touch_links=touch_links, pose=box_pose, size=(0.1, 0.1, 0.1))
# rospy.loginfo(wait_for_state_update(box_name, scene, box_is_attached=True, box_is_known=False))
# aco = AttachedCollisionObject()
# aco.object.id = "tool"
# aco.link_name = "mag_gripper_end_effector"
# scene.applyAttachedCollisionObject(aco)
