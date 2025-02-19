################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: OSRF ROS noetic Desktop Full
#       - https://hub.docker.com/r/osrf/ros/tags?page=1&name=noetic-desktop-full
################################################################################################

FROM osrf/ros:noetic-desktop-full AS base

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
    && rm -rf /var/lib/apt/lists/*

################################################################################################
# - ROS Installation stage
################################################################################################

FROM tools AS ros-dep

## Install necessary tools for imu
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-costmap-converter \
    && rm -rf /var/lib/apt/lists/*

################################################################################################
# - Final stage
#   - In this stage, I will install required packages for Navigation2.
#   - Reference:
#       - https://github.com/ros-navigation/navigation2
################################################################################################

FROM ros-dep AS final

USER ${USERNAME}
## Set working directory
WORKDIR /home/$USERNAME/localization-ws-ros1/src
RUN git clone https://github.com/tysik/obstacle_detector.git /home/$USERNAME/localization-ws-ros1/src/obstacle_detector

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