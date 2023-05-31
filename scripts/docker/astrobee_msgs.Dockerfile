# Copyright (c) 2021, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

# This will set up an Astrobee docker container using the non-NASA install instructions.
# You must set the docker context to be the repository root directory

ARG UBUNTU_VERSION=16.04
FROM ubuntu:${UBUNTU_VERSION}

ARG ROS_VERSION=kinetic
ARG PYTHON=""

# try to suppress certain warnings during apt-get calls
ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# install of apt-utils suppresses bogus warnings later
RUN apt-get update \
  && apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" \
  && apt-get install -y \
  build-essential \
  git \
  lsb-release \
  sudo \
  wget \
  && rm -rf /var/lib/apt/lists/*

# Install ROS

# this command is expected to have output on stderr, so redirect to suppress warning
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' >/dev/null 2>&1 \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update \
  && apt-get install -y \
  libtinyxml-dev \
  ros-${ROS_VERSION}-ros-base \
  python${PYTHON}-rosdep \
  python${PYTHON}-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

# Add the entrypoint for docker
RUN echo "#!/bin/bash\nset -e\n\nsource \"/opt/ros/${ROS_VERSION}/setup.bash\"\nexec \"\$@\"" > /ros_entrypoint.sh && \
  chmod +x /ros_entrypoint.sh && \
  rosdep init && \
  rosdep update 2>&1 | egrep -v 'as root|fix-permissions'


# Copy over the ff_msgs
COPY communications/ff_msgs /src/msgs/src/ff_msgs/ff_msgs/
COPY communications/ff_hw_msgs /src/msgs/src/ff_msgs/ff_hw_msgs/
