
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
    libgoogle-glog-dev libgflags-dev libgtest-dev \
    libluajit-5.1-dev \
    ros-rolling-xacro \
    ros-rolling-ros-testing \
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
