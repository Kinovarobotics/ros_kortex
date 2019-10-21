cmake_minimum_required(VERSION 2.8.3)

project(kortex_control)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

# Install
foreach(dir arms grippers)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
