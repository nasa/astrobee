
ARG UBUNTU_VERSION=20.04
ARG REMOTE=astrobee
FROM ${REMOTE}/astrobee:latest-rolling-ubuntu${UBUNTU_VERSION}

ARG ROS_DISTRO=rolling

COPY . /src/astrobee/src/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /src/astrobee \
    && CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/src/astrobee/src/cmake \
    && colcon build --symlink-install \
    && colcon test \
    && colcon test-result --verbose

COPY ./astrobee/resources /opt/astrobee/share/astrobee/resources
