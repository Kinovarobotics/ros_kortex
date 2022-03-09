search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=gen3_robotiq_2f_85.srdf
robot_name_in_srdf=gen3_robotiq_2f_85
moveit_config_pkg=gen3_robotiq_2f_85_moveit_config
robot_name=gen3_robotiq_2f_85
planning_group_name=arm
ikfast_plugin_pkg=gen3_robotiq_2f_85_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=tool_frame
ikfast_output_path=/home/alex/mor/kinova_dev_alex/ros_kortex/ik_fast/gen3_robotiq_2f_85_arm_ikfast_plugin/src/gen3_robotiq_2f_85_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
