
ARG UBUNTU_VERSION=20.04
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-rolling_base-ubuntu${UBUNTU_VERSION}

# install ros rolling + gazebo and dependencies
RUN apt-get update && apt-get install -q -y --fix-missing \
    ros-rolling-desktop \
    binutils \
    mesa-utils \
    x-window-system \
    ros-rolling-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*
