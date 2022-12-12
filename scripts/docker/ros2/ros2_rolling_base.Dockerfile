FROM ubuntu:focal

ENV ROSDISTRO=rolling
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# authorize GPG key and setup sources list
RUN apt-get update \
    && apt-get install -y \
        curl \
        gnupg \
        lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -y  \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest-timeout

# RUN mkdir -p /src/ros2_rolling_ws/src \
#     && cd /src/ros2_rolling_ws/ \
#     && vcs import --input https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos src \
#     && apt-get update && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" \
#     && colcon build --merge-install --install-base /opt/ros/rolling --cmake-args -DBUILD_TESTING=OFF --no-warn-unused-cli -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
#     && rm -rf /src/ros2_rolling_ws/ \
#     && rm -rf /var/lib/apt/lists/*

# # install gazebo and dependencies
# RUN apt-get update && apt-get install -q -y --fix-missing \
#     binutils \
#     mesa-utils \
#     x-window-system \
#     ros-rolling-gazebo-ros-pkgs \
#     && rm -rf /var/lib/apt/lists/*

# make python run python3 for compatibility with astrobee scripts
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1
