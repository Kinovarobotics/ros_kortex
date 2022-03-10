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

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ADD my_ros_entrypoint.sh /usr/bin/my_ros_entrypoint
RUN chmod +x /usr/bin/my_ros_entrypoint

ENTRYPOINT [ "my_ros_entrypoint" ]