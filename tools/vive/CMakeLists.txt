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

project(vive)

# This declares what ROS components our package needs

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    geometry_msgs
    ff_msgs
    ff_util
)

# Compile all c++ libraries and link into libvive.so

create_library(TARGET vive
  LIBS
    ${DEEPDIVE_LIBRARIES}
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ff_nodelet
    config_server
    msg_conversions
  INC
    ${DEEPDIVE_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  DEPS
    ff_msgs
)

# Build a small tool to interact with the nodelets

create_tool_targets(DIR tools
  LIBS
    ${catkin_LIBRARIES}
    ff_nodelet
  INC
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_msgs
)

# Install our launch files

install_launch_files()
