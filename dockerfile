FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get -y install python3-pip
RUN pip install conan
ENV CONAN_REVISIONS_ENABLED=1
RUN apt-get -y install ros-noetic-moveit && apt-get -y install ros-noetic-moveit-kinematics
RUN apt-get -y install ros-noetic-xacro
RUN apt-get -y install  ros-noetic-ros-controllers ros-noetic-ros-control
RUN apt-get install -y ros-noetic-octomap
RUN apt-get install -y ros-noetic-octomap ros-noetic-octomap-mapping
RUN apt-get -y install libzbar0 && pip install pyzbar
RUN pip install imutils
RUN apt-get update && apt-get -y install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base
RUN apt-get -y install ros-noetic-rgbd-launch

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /catkin_ws/src
COPY . /catkin_ws/src/ros_kortex

WORKDIR /catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
        && catkin_make"

# RUN source /opt/ros/noetic/setup.bash \
#     && catkin config --extend /opt/ros/noetic \
#     && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
#     && catkin_make

RUN echo 'source "/opt/ros/noetic/setup.bash"' >> ~/.bashrc \
    && echo 'source "/catkin_ws/devel/setup.bash"' >> ~/.bashrc

ADD my_ros_entrypoint.sh /usr/bin/my_ros_entrypoint
RUN chmod +x /usr/bin/my_ros_entrypoint

ENTRYPOINT [ "my_ros_entrypoint" ]