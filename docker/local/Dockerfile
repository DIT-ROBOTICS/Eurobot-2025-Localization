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
    && rm -rf /var/lib/apt/lists/*
    
################################################################################################
# - IMU Installation stage
#   - In this stage, install neccesary stuff for phidget IMU spatial.
################################################################################################

FROM tools AS imu-dep

## Install necessary tools for imu
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    ros-humble-imu-tools \
    libusb-1.0-0 libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

################################################################################################
# - RPLiDar Installation stage
#   - In this stage, install neccesary stuff for RPLiDar.
################################################################################################

FROM imu-dep AS rplidar-dep
RUN apt-get update && apt-get install -y \
    ros-humble-rplidar-ros \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    python3-numpy \
    git \
    && apt-get clean 

RUN pip3 install -U colcon-common-extensions
WORKDIR /home/user/localization-ws
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git /home/user/localization-ws/src/sensor-ws/rplidar-ros-driver/
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install
################################################################################################
# - obstacle detect Installation stage
# 
################################################################################################

FROM rplidar-dep AS obs-dep
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-laser-geometry \
    libarmadillo-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/user/localization-ws

RUN git clone -b obstacle-detect https://wintera1233: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@github.com/DIT-ROBOTICS/Eurobot-2025-Localization.git /tmp/repo \
    && mv /tmp/repo/localization-ws/src/obstacle_detector_2 /home/user/localization-ws/src/ && \
    rm -rf /tmp/repo


################################################################################################
# - Final stage
# 
################################################################################################

FROM obs-dep AS final

## Set working directory
WORKDIR /home/user/localization-ws

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
USER root
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /home/localization-ws/install/setup.bash" >> /etc/bash.bashrc
## Final configurationszz
USER $USERNAME
CMD ["/bin/bash"]