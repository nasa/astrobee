#!/bin/bash
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
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

set -euo pipefail

# apt-key is deprecated on Ubuntu 22.04+ and removed on 24.04+.
# Use scoped keyrings with signed-by (ROS signing key migration, 2024+).

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required to configure apt repositories." >&2
  exit 1
fi

. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"

sudo mkdir -p /etc/apt/keyrings

# ROS 1 repository (Melodic/Noetic on older Ubuntu; ros-apt-source on Jammy+)
if [ -f /etc/os-release ] && dpkg --compare-versions "${VERSION_ID:-0}" ge "22.04" 2>/dev/null; then
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | head -1 | awk -F\" '{print $4}')"
  curl -fsSL -o "/tmp/ros-apt-source_${ROS_APT_SOURCE_VERSION}.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  sudo dpkg -i "/tmp/ros-apt-source_${ROS_APT_SOURCE_VERSION}.deb"
  rm -f "/tmp/ros-apt-source_${ROS_APT_SOURCE_VERSION}.deb"
else
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /etc/apt/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu ${CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros-latest.list > /dev/null
fi

# Gazebo / OSRF repository
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /etc/apt/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
