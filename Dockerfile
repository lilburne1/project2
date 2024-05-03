FROM osrf/ros:humble-desktop as base

ENV ROS_DISTRO="humble"
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary software for the installation of ROS2
RUN apt-get update \
    && apt-get -y install \
    wget \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt install -y python3-pip\
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*rm 

# Setup scripts
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /workspaces
COPY ./project2 ./workspaces/project2

RUN git clone "https://github.com/reedhedges/AriaCoda.git"
RUN cd AriaCoda && make && make install
