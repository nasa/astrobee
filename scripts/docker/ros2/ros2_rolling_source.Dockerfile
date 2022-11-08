
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-rolling-base

ENV ROSDISTRO=rolling
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2


RUN mkdir -p /src/ros2_rolling_ws/src \
    && cd /src/ros2_rolling_ws/ \
    && vcs import --input https://raw.githubusercontent.com/ros2/ros2/rolling/ros2.repos src \
    && apt-get update && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" \
    && colcon build --merge-install --install-base /opt/ros/rolling --cmake-args -DBUILD_TESTING=OFF --no-warn-unused-cli -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
    && rm -rf /src/ros2_rolling_ws/ \
    && rm -rf /var/lib/apt/lists/*

# install gazebo and dependencies
RUN apt-get update && apt-get install -q -y --fix-missing \
    binutils \
    mesa-utils \
    x-window-system \
    ros-rolling-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*
