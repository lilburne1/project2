# Use the ROS Humble desktop image as the base
FROM osrf/ros:humble-desktop as base

# Set environment variables
ENV ROS_DISTRO="humble"
ENV DEBIAN_FRONTEND=noninteractive
ENV WS=/ws

# Set default shell to bash for the RUN commands
SHELL ["/bin/bash", "-c"]

# Install necessary software for the installation of ROS2 and additional dependencies
RUN apt-get update && apt-get -y install \
    wget \
    build-essential \
    cmake \
    git \
    libusb-1.0-0-dev \
    zsh \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joy \
    udev \
    doxygen \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install pip and setuptools
RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && pip3 install setuptools==58.2.0 && rm get-pip.py

# Install Oh My Zsh
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

# Setup the ROS workspace
RUN mkdir -p $WS/src
COPY ./project2/src/depthai-ros $WS/src/depthai-ros

# Install ROS dependencies
RUN source /opt/ros/humble/setup.bash && rosdep install --from-paths $WS/src --ignore-src -y

# Build the DepthAI ROS package
ARG BUILD_SEQUENTIAL=0
RUN source /opt/ros/humble/setup.bash && cd $WS && colcon build --symlink-install --packages-select depthai_ros -m 1 -r 1 -s $BUILD_SEQUENTIAL

# Optional RVIZ installation
ARG USE_RVIZ=0
RUN if [ "$USE_RVIZ" = "1" ] ; then \
    apt-get update && apt-get install -y ros-humble-rviz2 ros-humble-rviz-imu-plugin && rm -rf /var/lib/apt/lists/*; \
    else \
    echo "RVIZ NOT ENABLED"; \
    fi

# Update .zshrc and .bashrc for environment sourcing
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc

# Set the entry point and CMD
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
