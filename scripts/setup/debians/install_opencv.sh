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

# short help
usage_string="$scriptname [-h] [-p <install path>]"
#[-t make_target]

usage()
{
    echo "usage: sysinfo_page [[[-f file ] [-i]] | [-h]]"
}

while [ "$1" != "" ]; do
    case $1 in
        -p | --path )           shift
                                install_path=$1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

echo "Installing in: "${install_path:-/usr/local}

echo "Install the required dependencies"
sudo apt-get install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev

echo "Downloading OpenCV repo & switching to 3.3.1 branch"
mkdir ~/opencv_build && cd ~/opencv_build
git clone https://github.com/opencv/opencv.git
cd opencv && git checkout 3.3.1 && cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib && git checkout 3.3.1 && cd ..

echo "Building OpenCV"
cd ~/opencv_build/opencv
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=${install_path:-/usr/local} \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON \
    -D OPENCV_ENABLED_NONFREE=YES \
    -D ENABLE_PRECOMPILED_HEADERS=OFF ..

make -j8
echo "Installing OpenCV"
sudo make install
echo "Version installed:"
pkg-config --modversion opencv
