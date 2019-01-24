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

project(vive_localization)

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
    pluginlib
    sensor_msgs
    geometry_msgs
    nav_msgs
    ff_hw_msgs
    tf2_ros)

create_library(TARGET vive_localization
  DIR src
  LIBS ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} common config_reader msg_conversions ff_nodelet
  INC  ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPS common geometry_msgs sensor_msgs ff_hw_msgs)

create_tool_targets(
  DIR src/tool
  LIBS ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} vive_localization common config_reader
  INC  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
DEPS common geometry_msgs sensor_msgs ff_hw_msgs)

install_launch_files()

install(DIRECTORY config/ DESTINATION config)
