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

DIST=`cat /etc/os-release | grep -oP "(?<=VERSION_CODENAME=).*"`

if [ "$DIST" = "jammy" ]; then
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade -y

    echo "Ubuntu 22.04 detected, defaulting to install ROS2. Do you want full-desktop or base install?"
    echo "1) Full-desktop"
    echo "2) Base"
    read -p "Enter your choice: " choice
    case $choice in
        1)
            sudo apt install ros-humble-desktop -y
            sudo apt install ros-dev-tools -y

        ;;
        2)
            echo "Base install selected. You may need to install additional packages for your use case."
            sudo apt install ros-humble-base -y
            sudo apt install ros-dev-tools -y
        ;;
        *)
            echo "Invalid choice, exiting"
            exit 1 
        ;;
    esac
else
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    # Gazebo stuff
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add
fi
