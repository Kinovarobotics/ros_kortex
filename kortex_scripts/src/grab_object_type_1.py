#! /bin/python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import actionlib
from tbot_image_action_server.msg import QRScanActionMsgGoal, QRScanActionMsgResult, QRScanActionMsgFeedback, QRScanActionMsgAction
from tbot_image_action_server.msg import ObjectDetectActionMsgGoal, ObjectDetectActionMsgResult, ObjectDetectActionMsgFeedback, ObjectDetectActionMsgAction
import json
import string

joint_state_topic = ['joint_states:=/my_gen3/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('move_group_python', anonymous=True)

robot = moveit_commander.RobotCommander(robot_description="/my_gen3/robot_description")
scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3")    
arm = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
gripper = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="gripper")

qr_client = actionlib.SimpleActionClient('/qr_scan_as', QRScanActionMsgAction)
qr_client.wait_for_server()
print("connected to server")
od_client = actionlib.SimpleActionClient('/object_detect_as', ObjectDetectActionMsgAction)
od_client.wait_for_server()
print("connected to server")

def gripper_open(joints):
    joints[0] = 0
    gripper.set_joint_value_target(joints)
    gripper.plan()
    gripper.go(wait=True)
def gripper_close(joints):
    joints[0] = 0.3
    gripper.set_joint_value_target(joints)
    gripper.plan()
    gripper.go(wait=True)

def copy_pose(to_copy, adjustment):
    new_pose = geometry_msgs.msg.Pose()
    new_pose.position.x = to_copy.position.x + adjustment["x"]
    new_pose.position.y = to_copy.position.y + adjustment["y"]
    new_pose.position.z =to_copy.position.z + adjustment["z"]
    new_pose.orientation.x = to_copy.orientation.x + adjustment["X"]
    new_pose.orientation.y = to_copy.orientation.y + adjustment["Y"]
    new_pose.orientation.z = to_copy.orientation.z + adjustment["Z"]
    new_pose.orientation.w = to_copy.orientation.w + adjustment["W"]
    return new_pose
    
def make_adjustments(x, y, z, X, Y, Z, W):
    return {"x": x, "y": y, "z": z, "X": X, "Y": y, "Z": Z, "W": W}

def go_inital_pose():
    intital_pose = geometry_msgs.msg.Pose()
    intital_pose.orientation.w = 0.01112219699181503
    intital_pose.orientation.x = -0.00951493622721365
    intital_pose.orientation.y = -0.9998928588068303
    intital_pose.orientation.z = -0.00018338421950054657
    intital_pose.position.x = 0.21870236174029328
    intital_pose.position.y = -0.07132525403895153
    intital_pose.position.z = 0.8022677259334249
    arm.set_goal_tolerance(0.01)
    execute_shortest_plan(arm, intital_pose)
    # arm.set_pose_target(intital_pose)
    # arm.plan()
    # arm.go(wait=True)  

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

def od_result_callback(state, result):
    if state == 3:
        qr_goal.num_codes = result.result
    else:
        qr_goal.num_codes = 0

def qr_result_callback(state, result):
    #print(result)
    global foo
    if state != 3:
        foo = 2
        return
    res = json.loads(result.result)
    x_p_ind = res["pepper"]["polygon"]["point_a"].find(",")
    x_pepper = int(res["pepper"]["polygon"]["point_a"][:x_p_ind].strip(string.ascii_letters + "()=,"))
    x_t_ind = res["tomato"]["polygon"]["point_a"].find(",")
    x_tomato = int(res["tomato"]["polygon"]["point_a"][:x_t_ind].strip(string.ascii_letters + "()=,"))
    print("pepper: " + str(x_pepper) + ", tomoto: " + str(x_tomato))
    if (x_pepper < x_tomato):
        foo = 1
    else: 
        foo = 0

pose_target_1 = geometry_msgs.msg.Pose()
pose_target_1.position.x = 0.369913922
pose_target_1.position.y = -0.01035380
pose_target_1.position.z = 0.514513493
pose_target_1.orientation.x = -0.706459
pose_target_1.orientation.y = -0.707715
pose_target_1.orientation.z = 0.0045551
pose_target_1.orientation.w = 0.0056552

p_z = make_adjustments(0, 0, 0.2, 0, 0, 0, 0)
n_x = make_adjustments(-0.1, 0, 0, 0, 0, 0, 0)

pose_target_1_up = copy_pose(pose_target_1, p_z)
pose_target_2 = copy_pose(pose_target_1, n_x)
pose_target_2_up = copy_pose(pose_target_2, p_z)
pose_target_3 = copy_pose(pose_target_2, n_x)
pose_target_3_up = copy_pose(pose_target_3, p_z)
pose_target_4 = copy_pose(pose_target_3, n_x)
pose_target_4_up = copy_pose(pose_target_4, p_z)

pose_list = [pose_target_1_up, pose_target_1, pose_target_1_up, pose_target_2_up, pose_target_2, pose_target_2_up, pose_target_3_up, pose_target_3, pose_target_3_up, pose_target_4_up, pose_target_4]
joints = gripper.get_current_joint_values()

#arm.clear_trajectory_constraints()
#pcm = moveit_msgs.msg.OrientationConstraint()
#pcm.link_name = arm.get_end_effector_link()
#pcm.absolute_x_axis_tolerance = 3.14 # ignore errors in the x-axis
#pcm.absolute_y_axis_tolerance = 3.14 # ignore errors in the x-axis
#pcm.absolute_z_axis_tolerance =  1.57 # enforce z-axis orientation
#pcm.weight = 1000000
#path_constraints = moveit_msgs.msg.Constraints()
#path_constraints.orientation_constraints.append(pcm)
#arm.set_path_constraints(path_constraints)
while True:
    go_inital_pose()
    #gripper_open(joints)

    qr_goal = QRScanActionMsgGoal()
    od_goal = ObjectDetectActionMsgGoal()
    od_goal.path = "/my_gen3/camera/depth/image_raw"
    od_goal.time_out = 1.0
    od_goal.threshold = 0.06
    od_client.send_goal(od_goal, done_cb=od_result_callback)
    od_client.wait_for_result()

    qr_goal.path = "/my_gen3/camera/color/image_raw"
    qr_goal.data = ""
    qr_goal.time_out = 1.0
    print(qr_goal.num_codes)
    qr_client.send_goal(qr_goal, done_cb=qr_result_callback)
    print("sent goal")
    qr_client.wait_for_result()

    print(foo)
    time.sleep(1)
    if foo == 1:
        
        fraction = 0
        arm.set_goal_tolerance(0.01)
        # while fraction<1.0:
        #     waypoints = [pose_target_1_up, pose_target_1, ]
        #     (plan, fraction) = arm.compute_cartesian_path(
        #         waypoints, 0.01, 0  # waypoints to follow  # eef_step
        #     )  # jump_threshold 
        #     print("fraction: " + str(fraction))

        # print("fraction: " + str(fraction))
        # arm.execute(plan, wait=True)
        execute_shortest_plan(arm, pose_target_1_up)
        execute_shortest_plan(arm, pose_target_1)
        time.sleep(1)
        #gripper_close(joints)
        time.sleep(1)
        
        # waypoints = [ pose_target_1_up, pose_target_4_up, pose_target_4]
        # fraction = 0
        # while fraction<1.0:
        #     (plan, fraction) = arm.compute_cartesian_path(
        #     waypoints, 0.01, 0  # waypoints to follow  # eef_step
        #     )  # jump_threshold 
        #     print("fraction: " + str(fraction))
        # print("fraction: " + str(fraction))
        # arm.execute(plan, wait=True)
        execute_shortest_plan(arm, pose_target_1_up)
        execute_shortest_plan(arm, pose_target_4_up)
        execute_shortest_plan(arm, pose_target_4)
        time.sleep(1)
        #gripper_open(joints)

    elif foo == 0:
        
        # waypoints = [pose_target_2_up, pose_target_2]
        # fraction = 0
        # while fraction<1.0:
        #     (plan, fraction) = arm.compute_cartesian_path(
        #         waypoints, 0.01, 0  # waypoints to follow  # eef_step
        #     )  # jump_threshold 
        #     print("fraction: " + str(fraction))
        # print("fraction: " + str(fraction))
        # arm.execute(plan, wait=True)
        execute_shortest_plan(arm, pose_target_2_up)
        execute_shortest_plan(arm, pose_target_2)
        time.sleep(1)
        #gripper_close(joints)
        time.sleep(1)
        
        # waypoints = [ pose_target_2_up, pose_target_4_up, pose_target_4]
        # fraction = 0
        # while fraction<1.0:
        #    (plan, fraction) = arm.compute_cartesian_path(
        #     waypoints, 0.01, 0  # waypoints to follow  # eef_step
        #     )  # jump_threshold
        # print("fraction: " + str(fraction))
        # arm.execute(plan, wait=True)
        execute_shortest_plan(arm, pose_target_2_up)
        execute_shortest_plan(arm, pose_target_4_up)
        execute_shortest_plan(arm, pose_target_4)
        time.sleep(1)
        #gripper_open(joints)

    else: 
        print("scan failed aborting to inital state")


moveit_commander.roscpp_shutdown()
