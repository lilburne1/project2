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


# Setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["ros_entrypoint.sh"]
CMD ["bash"]

WORKDIR /workspaces
COPY ./project2 ./project2

RUN git clone "https://github.com/reedhedges/AriaCoda.git"
RUN cd AriaCoda && make && make install

WORKDIR /workspaces/project2

RUN source /opt/ros/humble/setup.bash && \
    colcon build && \
    source ./install/setup.bash

