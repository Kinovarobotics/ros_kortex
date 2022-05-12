FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get -y install python3-pip
RUN pip install conan
ENV CONAN_REVISIONS_ENABLED=1
RUN apt-get -y install ros-noetic-moveit && apt-get -y install ros-noetic-moveit-kinematics
RUN apt-get -y install ros-noetic-xacro
RUN apt-get -y install  ros-noetic-ros-controllers ros-noetic-ros-control
RUN apt-get -y install ros-noetic-octomap-rviz-plugins
RUN apt-get -y install libzbar0 && pip install pyzbar
RUN pip install imutils
RUN apt-get -y install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base
RUN apt-get -y install ros-noetic-rgbd-launch

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# RUN mkdir catkin_ws
# RUN mkdir catkin_ws/src
# ADD ros_kortex /catkin_ws/src
# RUN source /opt/ros/noetic/setup.bash
# RUN catkin_make

ADD my_ros_entrypoint.sh /usr/bin/my_ros_entrypoint
RUN chmod +x /usr/bin/my_ros_entrypoint

ENTRYPOINT [ "my_ros_entrypoint" ]