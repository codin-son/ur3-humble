# Docker for ur3 repo
# ros-humble-desktop-full, gazebo11 (classic), python3 libraries
# ROS2 Humble Hawksbill — Ubuntu 22.04

FROM osrf/ros:humble-desktop-full
LABEL maintainer="Cristian Beltran <beltran@hlab.sys.es.osaka-u.ac.jp>"

ENV ROS_DISTRO humble

# Install ROS2 packages and utilities
RUN apt-get update && apt-get install -y \
    # Gazebo Classic (gazebo11) for Humble
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    # ros2_control
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    # MoveIt2
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-ros-planning-interface \
    ros-$ROS_DISTRO-moveit-simple-controller-manager \
    # TF2
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf-transformations \
    # Robot description
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-xacro \
    # RViz2
    ros-$ROS_DISTRO-rviz2 \
    # SpaceNav
    ros-$ROS_DISTRO-spacenav \
    # RQT
    ros-$ROS_DISTRO-rqt-common-plugins \
    # PyKDL
    python3-pykdl \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    # Utilities
    git \
    locate \
    vim htop \
    curl wget \
    spacenavd \
    && rm -rf /var/lib/apt/lists/*

### ROS2 Workspace ###
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
               && mkdir -p ~/ros2_ws/src \
               && cd ~/ros2_ws/src \
               && git clone https://github.com/cambel/ur3.git ros-ur -b humble-devel"

# Install Python3 dependencies
ENV PYTHONIOENCODING UTF-8

RUN python3 -m pip install --upgrade pip && \
    pip3 install \
    matplotlib \
    scipy \
    protobuf \
    pyyaml \
    pyquaternion \
    lxml \
    tqdm \
    empy \
    PyVirtualDisplay \
    defusedxml \
    psutil \
    pyprind \
    transforms3d

# Update rosdep and install dependencies
RUN cd ~/ros2_ws \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y \
    && rm -rf /var/lib/apt/lists/*

# Build workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
               && cd ~/ros2_ws/ \
               && colcon build --symlink-install"

# Download Gazebo models
RUN mkdir -p ~/.gazebo
RUN git clone https://github.com/osrf/gazebo_models ~/.gazebo/models

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

EXPOSE 11345

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/ros-ur/ur_gripper_gazebo/models/' >> ~/.bashrc
