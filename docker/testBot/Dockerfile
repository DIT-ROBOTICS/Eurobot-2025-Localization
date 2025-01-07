################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: OSRF ROS Noetic Desktop Full
#       - https://hub.docker.com/r/osrf/ros/tags?page=1&name=noetic-desktop-full
################################################################################################

FROM osrf/ros:noetic-desktop-full AS base

LABEL org.opencontainers.image.authors="yoseph.huang@gmail.com"

ARG USERNAME=user
ARG USER_UID=1004
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]

################################################################################################
# - User Setup stage
#   - In this stage, create a non-root user and configure passwordless sudo.
################################################################################################

FROM base AS user-setup

# Create a non-root user
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

# Install necessary packages
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
    libsdl-image1.2-dev \
    libsdl-dev \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-mbf-costmap-core \
    ros-noetic-mbf-msgs \
    libsuitesparse-dev \
    ros-noetic-libg2o \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-costmap-converter \
    ros-noetic-robot-localization \
    ros-noetic-imu-tools \
    libusb-1.0-0 libusb-1.0-0-dev -y && \
    rm -rf /var/lib/apt/lists/*

################################################################################################
# - RPLiDar Installation stage
#   - In this stage, install necessary tools for RPLiDar.
################################################################################################

FROM tools AS rplidar-dep
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    python3-numpy \
    git && \
    apt-get clean

WORKDIR /home/${USERNAME}/localization-ws-ros1
RUN git clone https://github.com/Slamtec/rplidar_ros.git /home/${USERNAME}/localization-ws-ros1/src/rplidar-ros-driver
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

################################################################################################
# - Obstacle detect Installation stage
################################################################################################

FROM rplidar-dep AS obs-dep

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-laser-geometry \
    libarmadillo-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USERNAME}/localization-ws-ros1
RUN mkdir -p /home/${USERNAME}/localization-ws-ros1/src/localization-devel-ws &&  \
    git clone https://github.com/tysik/obstacle_detector.git /home/${USERNAME}/localization-ws-ros1/src/localization-devel-ws/obstacle_detector

RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
USER root
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc && \
    echo "source /home/${USERNAME}/localization-ws-ros1/devel/setup.bash" >> /etc/bash.bashrc
# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

################################################################################################
# - Spatial-driver Installation stage
################################################################################################

FROM obs-dep AS imu-dep
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    ros-noetic-imu-tools \
    libusb-1.0-0 libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*
COPY phidgets_drivers /home/${USERNAME}/localization-ws-ros1/src/phidgets_drivers

################################################################################################
# - Final stage
################################################################################################

FROM imu-dep AS final

## Set working directory
WORKDIR /home/${USERNAME}/localization-ws-ros1

# Ensure that the ROS workspace is built and set up correctly
RUN source /opt/ros/noetic/setup.bash && \
    cd /home/${USERNAME}/localization-ws-ros1 && \
    catkin_make

USER ${USERNAME}
SHELL ["/bin/bash", "-c"]

CMD ["/bin/bash"]