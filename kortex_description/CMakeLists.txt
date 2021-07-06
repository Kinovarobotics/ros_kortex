cmake_minimum_required(VERSION 3.0.2)

project(kortex_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

# Install
foreach(dir arms grippers robots launch)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
