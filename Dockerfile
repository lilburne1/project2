FROM osrf/ros:humble-desktop as base

ENV ROS_DISTRO="humble"
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install necessary software for the installation of ROS2
RUN apt-get update \
    && apt-get -y install \
    wget \
    build-essential \
    cmake \
    x11-apps \
    x11-xserver-utils \
    x11-utils \
    ros-humble-tf2-tools \
    ros-humble-rqt-graph \
    ros-humble-sick-scan-xd \
    ros-humble-phidgets-spatial \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-depthai-ros \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \ 
    ros-humble-joy \
    ros-humble-rviz2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && pip3 install setuptools==58.2.0 && rm get-pip.py
RUN apt-get update && apt-get install -y udev
RUN apt-get update && apt-get install -y doxygen


# Setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "xhost +local:" >> /root/.bashrc

WORKDIR /workspaces

RUN git clone "https://github.com/reedhedges/AriaCoda.git"
RUN cd AriaCoda && make && make install

COPY ./project2 ./project2
WORKDIR /workspaces/project2

# Ensuring the ROS environment is properly sourced before building
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Set the entry point and permissions
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

