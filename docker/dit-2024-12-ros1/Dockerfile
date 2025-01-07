################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: OSRF ROS noetic Desktop Full
#       - https://hub.docker.com/r/osrf/ros/tags?page=1&name=noetic-desktop-full
################################################################################################

FROM osrf/ros:noetic-desktop-full AS base

LABEL org.opencontainers.image.authors="jossiew621@gapp.nthu.edu.tw"

ARG USERNAME=localization
ARG USER_UID=1004
ARG USER_GID=$USER_UID
ARG WORKSPACE_DIR=/home/$USERNAME/localization-ws

SHELL ["/bin/bash", "-c"]
ENV TERM=xterm-256color

################################################################################################
# - User Setup stage
#   - In this stage, create a non-root user and configure passwordless sudo.
################################################################################################

FROM base AS user-setup

## Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash && \
    apt-get update && \
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*


################################################################################################
# - Tools Installation stage
#   - In this stage, I will install convenient tools for development.
################################################################################################

FROM user-setup AS tools

## Install necessary packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    apt-utils \
    git \
    nano \
    net-tools \
    ssh \
    usbutils \
    udev \ 
    x11-apps \
    tmux \
    htop \ 
    && rm -rf /var/lib/apt/lists/*

################################################################################################
# - LiDAR Installation stage
#   - In this stage, install neccesary stuff for phidget IMU spatial.
################################################################################################

FROM tools AS lidar-dep

## Install necessary tools for lidar
WORKDIR /home/YDLidar-SDK

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git --branch master /home/YDLidar-SDK && \
    cd /home/YDLidar-SDK && \
    git checkout 7c0a2328fd85b65960c37287a67b0edbee8caa4b
RUN sudo mkdir -p build && rm -rf build/*
WORKDIR /home/YDLidar-SDK/build
RUN sudo cmake .. && \
    sudo make && \
    sudo make install

USER ${USERNAME}
WORKDIR ${WORKSPACE_DIR}/src/sensor-ws/ydlidar_ros_driver
RUN git clone https://github.com/jossiewang/ydlidar_ros_driver-G6.git ${WORKSPACE_DIR}/src/sensor-ws/ydlidar_ros_driver
RUN git clone https://github.com/tysik/obstacle_detector.git 

RUN sudo apt-get autoremove -y && \
    sudo apt-get autoclean -y && \
    sudo rm -rf /var/lib/apt/lists/*

################################################################################################
# - IMU Installation stage
#   - In this stage, install neccesary stuff for phidget IMU spatial.
################################################################################################

# FROM tools AS imu-dep

# ## Install necessary tools for imu
# RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -y \
#     python3-pip \
#     ros-noetic-imu-tools \
#     libusb-1.0-0 libusb-1.0-0-dev \
#     && rm -rf /var/lib/apt/lists/*

################################################################################################
# - Final stage
#   - In this stage, I will install required packages for Navigation2.
#   - Reference:
#       - https://github.com/ros-navigation/navigation2
################################################################################################

FROM lidar-dep AS final

## Set working directory
WORKDIR ${WORKSPACE_DIR}

USER ${USERNAME}
COPY .bashrc /home/$USERNAME/.bashrc.conf
RUN cat /home/$USERNAME/.bashrc.conf >> /home/$USERNAME/.bashrc

## NOTE:
## Steps below should ideally be performed in a separate container and then mounted as a volume to the main container, 
## as they require modifications that are better handled outside the Dockerfile.

# RUN apt-get update && \
#     git clone https://github.com/ros-planning/navigation2.git --branch $ROS_DISTRO src/navigation2 && \
#     rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
#     /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install' \
#     && rm -rf /var/lib/apt/lists/*


## Final configurations
USER $USERNAME
CMD ["/bin/bash"]