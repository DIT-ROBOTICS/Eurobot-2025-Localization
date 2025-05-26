################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: OSRF ROS Humble Desktop Full
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
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash && \
    apt-get update && \
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*
################################################################################################
# - Tools Installation stage
################################################################################################
FROM user-setup AS tools
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
    libsdl-image1.2-dev \
    libarmadillo-dev \
    libboost-all-dev \
    libsuitesparse-dev \
    ros-humble-libg2o \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
################################################################################################
# - Source clone stage
################################################################################################
FROM tools AS source
USER $USERNAME
WORKDIR /home/$USERNAME/localization-ws/src
# === Clone required packages ===
WORKDIR /home/user/localization-ws/src/YDLidar-SDK
RUN git clone https://github.com/wintera1233/YDLidar-SDK.git /home/user/localization-ws/src/YDLidar-SDK/
WORKDIR /home/user/localization-ws/src/localization-devel-ws/rival_localization
RUN cd /home/user/localization-ws/src/YDLidar-SDK/build && \
    cmake .. && \
    make -j4 && \
    sudo make install
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git /home/user/localization-ws/src/rplidar_ros \
 && git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/user/localization-ws/src/ydlidar_ros2_driver
RUN git clone -b sideLidar https://github.com/DIT-ROBOTICS/Eurobot-2025-Localization.git \
 && mv Eurobot-2025-Localization/localization-devel-ws/rival/rival_localization /home/$USERNAME/localization-ws/src/localization-devel-ws/rival_localization \
 && mv Eurobot-2025-Localization/localization-devel-ws/global/obstacle_detector_2-humble-devel /home/$USERNAME/localization-ws/src/localization-devel-ws/obstacle_detector_2-humble-devel \
 && rm -rf Eurobot-2025-Localization

RUN 
################################################################################################
# - Final Build Stage
################################################################################################
FROM source AS final
WORKDIR /home/$USERNAME/localization-ws
USER $USERNAME
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install" && \
    sudo rm -rf /var/lib/apt/lists/*

RUN rm -rf /home/$USERNAME/localization-ws/src/localization-devel-ws

COPY ./dds.xml /tmp/dds.xml
# COPY .bashrc /home/$USERNAME/.bashrc.conf
# RUN cat /home/$USERNAME/.bashrc.conf >> /home/$USERNAME/.bashrc

#first build: wintera1233/side_lidar:test