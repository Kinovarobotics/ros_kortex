#! /bin/python3

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
group = moveit_commander.MoveGroupCommander(robot_description="my_gen3/robot_description", ns="/my_gen3", name="arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
group.allow_replanning(True)
group.num_planning_attempts = 10
group.replan_attempts = 10
group.set_goal_tolerance(0.01)
group.set_planner_id("RRTConnect")
def execute_shortest_plan(group, pose_target):
  group.set_pose_target(pose_target)
  plans = []
  points = []

  for i in range(10):
    plans.append(list(group.plan()))
    points.append(len(plans[i][1].joint_trajectory.points))
  
  shortest_plan =  min(range(len(points)), key=points.__getitem__)
  print("number of waypoints: " + str(points) )
  print("shortest plan index: " + str(shortest_plan))

  group.execute(plans[shortest_plan][1], wait=True)

def reach_cartesian_pose(pose, tolerance, constraints):
   
    
    # Set the tolerance
    group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return group.go(wait=True)

planning_time = 1
x = 0
for x in range(5):
  group.clear_trajectory_constraints()
  print("planning time: " + str(planning_time/(x+1)))
  group.set_planning_time(planning_time/(x+1))
  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = 0.20228798672323686
  pose_target.position.y = -0.0844410319119024
  pose_target.position.z = 0.8287765803854372
  pose_target.orientation.x = 0.004293068374565495
  pose_target.orientation.y = 0.9999850315352882
  pose_target.orientation.z = 0.0033686968829585847
  pose_target.orientation.w = 0.0003976815453087705

  #print(type(group))
  #print(type(pose_target))
  execute_shortest_plan(group, pose_target)

  group.clear_trajectory_constraints()
  # current_pose = group.get_current_pose()
  # pcm = moveit_msgs.msg.OrientationConstraint()
  # pcm.link_name = group.get_end_effector_link()
  # pcm.absolute_x_axis_tolerance = 0.2 # ignore errors in the x-axis
  # pcm.absolute_y_axis_tolerance = 0.2 # ignore errors in the x-axis
  # pcm.absolute_z_axis_tolerance =  3.14 # enforce z-axis orientation
  # pcm.header.frame_id = group.get_planning_frame()
  # pcm.weight = 1
  # pcm.orientation = current_pose.pose.orientation
  # print(pcm.orientation)
  # path_constraints = moveit_msgs.msg.Constraints()
  # path_constraints.name = group.get_end_effector_link()
  # path_constraints.orientation_constraints.append(pcm)
  # group.set_path_constraints(path_constraints)

  # actual_pose = group.get_current_pose().pose
  # actual_pose.position.y -= 0.3

  # # Orientation constraint (we want the end effector to stay the same orientation)
  # constraints = moveit_msgs.msg.Constraints()
  # orientation_constraint = moveit_msgs.msg.OrientationConstraint()
  # orientation_constraint.orientation = actual_pose.orientation
  # constraints.orientation_constraints.append(orientation_constraint)

  # # Send the goal
  # reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
  # #plan_list = list(plan1)
  #print(dir(plan_list[1].joint_trajectory.points))
  #print(len(plan_list[1].joint_trajectory.points))

  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = 0.20228798672323686
  pose_target.position.y = -0.2844410319119024
  pose_target.position.z = 0.8287765803854372
  pose_target.orientation.x = 0.004293068374565495
  pose_target.orientation.y = 0.9999850315352882
  pose_target.orientation.z = 0.0033686968829585847
  pose_target.orientation.w = 0.0003976815453087705
  execute_shortest_plan(group, pose_target)
  #break

moveit_commander.roscpp_shutdown()
