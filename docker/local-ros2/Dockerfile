################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: OSRF ROS Humble Desktop Full
#       - https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble-desktop-full
################################################################################################

FROM osrf/ros:humble-desktop-full AS base

LABEL org.opencontainers.image.authors="yoseph.huang@gmail.com"

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]


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
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    python3-numpy \
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
    ros-humble-foxglove-bridge \
    libsdl-image1.2-dev \
    libarmadillo-dev \
    libboost-all-dev \
    # ros-humble-laser-geometry \
    libsuitesparse-dev \
    ros-humble-libg2o \
    # ros-humble-imu-tools \
    libusb-1.0-0 libusb-1.0-0-dev -y && \
    rm -rf /var/lib/apt/lists/*
    
################################################################################################
# - IMU Installation stage
#   - In this stage, install neccesary stuff for phidget IMU spatial.
################################################################################################

FROM tools AS imu-dep

## Install necessary tools for imu
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-imu-tools \
    && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
WORKDIR /home/user/localization-ws/src/phidgets_drivers
COPY phidgets_drivers /home/user/localization-ws/src/phidgets_drivers

################################################################################################
# - RPLiDar and obstacle detector Installation stage
#   - In this stage, install neccesary stuff for RPLiDar.
################################################################################################

FROM imu-dep AS lidar-dep
USER ${USERNAME}
WORKDIR /home/user/localization-ws/src/rplidar-ros-driver
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git /home/user/localization-ws/src/rplidar-ros-driver/
RUN git clone -b humble-devel https://github.com/harmony-eu/obstacle_detector_2.git /home/user/localization-ws/src/obstacle_detector_2

################################################################################################
# - Final stage
# 
################################################################################################

FROM lidar-dep AS final

## Set working directory
WORKDIR /home/${USERNAME}/localization-ws
# RUN sudo chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/localization-ws
USER ${USERNAME}
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install' \
    && sudo rm -rf /var/lib/apt/lists/*

COPY .bashrc /home/$USERNAME/.bashrc.conf
RUN cat /home/$USERNAME/.bashrc.conf >> /home/$USERNAME/.bashrc

USER ${USERNAME}
CMD ["/bin/bash"]