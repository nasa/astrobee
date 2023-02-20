
ARG UBUNTU_VERSION=20.04
ARG REMOTE=astrobee
FROM osrf/ros:humble-desktop

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# install gazebo and dependencies
RUN apt-get update && apt-get install -q -y --fix-missing \
    binutils \
    mesa-utils \
    x-window-system \
    libgoogle-glog-dev libgflags-dev libgtest-dev \
    libluajit-5.1-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-ros-testing \
    ros-humble-launch-pytest \
    && rm -rf /var/lib/apt/lists/*

# Install Astrobee----------------------------------------------------------------
COPY ./scripts/setup/debians /setup/astrobee/debians

RUN apt-get update \
  && /bin/bash /setup/astrobee/debians/build_install_debians.sh \
  && rm -rf /var/lib/apt/lists/* \
  && rm -rf /setup/astrobee/debians

# COPY ./scripts/setup/packages_*.lst /setup/astrobee/
# note apt-get update is run within the following shell script
# RUN /setup/astrobee/install_desktop_packages.sh \
#   && rm -rf /var/lib/apt/lists/*
