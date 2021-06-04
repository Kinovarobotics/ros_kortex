#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal

# Get arguments
parser = argparse.ArgumentParser(description='Launch ROS examples or ROS driver automated tests for a given arm configuration.')
parser.add_argument('--no-vision', action='store_true', help='Add this option if the arm doesn\'t have a vision module')
parser.add_argument('ip', type = str, help='Arm IP address (enter \'none\' if testing Gazebo)')
parser.add_argument('arm', choices = ['gen3', 'gen3_lite'], help='Arm type')
parser.add_argument('dof', type = int, help='Number of DOF for the arm')
parser.add_argument('gripper', choices = ['robotiq_2f_85', 'robotiq_2f_140', 'gen3_lite_2f', 'none'], help='Gripper type')
parser.add_argument('test_case', choices = ['driver', 'examples', 'gazebo'], help='Test case to run')

args = parser.parse_args()

# Create the command for the driver
command = []
if args.test_case == 'driver' or args.test_case == 'examples':
    if args.test_case == 'driver':
        command.append("rostest")
        command.append("--reuse-master")
    elif args.test_case == 'examples':
        command.append("roslaunch")
    command.append("kortex_driver")
    command.append("kortex_driver.launch")
    command.append("ip_address:={}".format(args.ip))
    command.append("start_moveit:=true")
elif args.test_case == 'gazebo':
    command.append("roslaunch")
    command.append("kortex_gazebo")
    command.append("spawn_kortex_robot.launch")
    command.append("gazebo_gui:=false")
    command.append("start_delay_seconds:=20") #longer load time for Gazebo in Docker
command.append("start_rviz:=false")
command.append("arm:={}".format(args.arm))
command.append("dof:={}".format(args.dof))
# Add vision option
if args.no_vision:
    command.append("vision:=false")
else:
    command.append("vision:=true")
# Add gripper option
if args.gripper != 'none':
    command.append("gripper:={}".format(args.gripper))

try:
    ros_master_process = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL)
    time.sleep(3)
    # If we want to run the tests for the driver
    if args.test_case == 'driver':
        p = subprocess.Popen(command)
        p.wait()
    # If we want to test the examples
    elif args.test_case == 'examples':
        p = subprocess.Popen(command, stdout=subprocess.DEVNULL)
        print ("Starting the ROS Kortex driver...")
        time.sleep(10)
        examples_tests_process = subprocess.Popen(['rostest', '--reuse-master', 'kortex_examples', 'run_all_examples.launch', 'robot_name:=my_{}'.format(args.arm)])
        examples_tests_process.wait()
    elif args.test_case == 'gazebo':
        p = subprocess.Popen(command)
        p.wait()
except KeyboardInterrupt:
    p.terminate()
finally:
    os.killpg(os.getpid(), signal.SIGTERM)
