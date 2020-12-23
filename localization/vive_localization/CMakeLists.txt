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

cmake_minimum_required(VERSION 2.8.12) # Need get_filename_component DIRECTORY
project(vive_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  nodelet
  geometry_msgs
  ff_hw_msgs
  ff_util
  msg_conversions
  config_reader
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find OpenCV3
find_package(OpenCV 3 REQUIRED)

# Non-linear optimizer
find_package(Ceres REQUIRED)

# What this project provides
catkin_package(
  LIBRARIES
    vive_localization
  CATKIN_DEPENDS
    roscpp
    rosbag
    nodelet
    ff_hw_msgs
    ff_util
    msg_conversions
    config_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(vive_localization
  src/vive.cc
  src/vive_solver.cc
  src/tool/vive_offline.cc
)
add_dependencies(vive_localization ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_localization json-c usb-1.0 z ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/ DESTINATION config)
