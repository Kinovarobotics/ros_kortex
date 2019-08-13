This ROS package was cloned from https://github.com/waypointrobotics/robotiq_85_gripper into ros_kortex to use the description files for the Robotiq 2f 85 gripper. Only the robotiq_85_description folder was copied (along with this readme and the LICENSE file from the original repo).
The repository was cloned at commit 7e09b0d4d90179cf4809193914c7f6d44a9140a6.

The xacro files were modified to allow proper simulation of the gripper in Gazebo. 

The original readme file follows:

# robotiq_85_gripper
Common packages for the Robotiq 85 Gripper provided by Stanley Innovation

Defaults to 'ttyUSB0' and 115200 baud rate

Single gripper and left gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface
Right gripper (in dual gripper configuration) need to be configured as device 9 using the Robotiq User Interface


start with:
roslaunch robotiq_85_bringup robotiq_85.launch run_test:=true