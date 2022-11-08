FROM astrobee:latest-rolling-base

# install ros rolling + gazebo and dependencies
RUN apt-get update && apt-get install -q -y --fix-missing \
    ros-rolling-desktop \
    binutils \
    mesa-utils \
    x-window-system \
    ros-rolling-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*
