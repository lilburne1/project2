FROM osrf/ros:humble-desktop as base

ENV ROS_DISTRO="humble"
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary software for the installation of ROS2
RUN apt-get update \
    && apt-get -y install \
    wget \
    build-essential \
    cmake \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && pip3 install setuptools==58.2.0 && rm get-pip.py
RUN apt-get update && apt-get install -y udev
RUN apt-get update && apt-get install -y doxygen

# install -y python3-pip\
#     python3-doxygen
#     # python3-colcon-common-extensions \
#     # python3-rosdep \
#     # python3-argcomplete \
#     # && rm -rf /var/lib/apt/lists/*rm 

# Setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc


# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /workspaces
COPY ./project2 ./project2

WORKDIR /workspaces/project2/src
RUN git clone "https://github.com/reedhedges/AriaCoda.git"
RUN cd AriaCoda && make && make install

WORKDIR /workspaces/project2

RUN colcon build
RUN echo "source ./install/setup.bash"

WORKDIR /workspaces/project2
