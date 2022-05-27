FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get -y install python3-pip
RUN pip install \
    conan \
    pyzbar \
    imutils
ENV CONAN_REVISIONS_ENABLED=1
RUN apt-get update && apt-get -y install \
    ros-noetic-moveit && apt-get -y install ros-noetic-moveit-kinematics \
    ros-noetic-xacro \
    ros-noetic-ros-controllers ros-noetic-ros-control \
    ros-noetic-octomap \
    ros-noetic-octomap ros-noetic-octomap-mapping \
    libzbar0 \
    ros-noetic-rgbd-launch \
    ros-noetic-catkin python3-catkin-tools \
    git
RUN apt-get update && apt-get -y install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base
RUN apt-get update && apt-get -y upgrade

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /catkin_ws/src
COPY . ../catkin_ws/src/ros_kortex

WORKDIR /catkin_ws

RUN cd src && git clone https://github.com/ecoation-labs/ros_kortex_vision.git

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
        && catkin build"

RUN echo 'source "/opt/ros/noetic/setup.bash"' >> ~/.bashrc \
    && echo 'source "/catkin_ws/devel/setup.bash"' >> ~/.bashrc

ADD my_ros_entrypoint.sh /usr/bin/my_ros_entrypoint
RUN chmod +x /usr/bin/my_ros_entrypoint

ENTRYPOINT [ "my_ros_entrypoint" ]