cmake_minimum_required(VERSION 3.0.2)

project(kortex_control)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

# Install
foreach(dir arms grippers)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
