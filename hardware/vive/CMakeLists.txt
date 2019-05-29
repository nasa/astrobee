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

# Declare the project
project(vive)

# We need standard math
find_library(CMATH_LIBRARIES NAMES m)

# declare the catkin package
catkin_package(
  LIBRARIES
    vive_driver
    vive_nodelet
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    ff_hw_msgs
)

# ROS agnostic proxy library and tool

create_library(
  DIR src/vive_driver
  TARGET vive_driver
  INC ${USB_INCLUDE_DIRS} ${JSONC_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS}
  LIBS ${USB_LIBRARIES} ${JSONC_LIBRARIES} ${ZLIB_LIBRARIES} ${CMATH_LIBRARIES}
)

create_tool_targets(
  DIR tools/vive_tool
  INT ${ARGTABLE2_INCLUDE_DIRS}
  LIBS vive_driver ${ARGTABLE2_LIBRARIES}
)

# ROS nodelet

create_library(
  DIR src/vive_nodelet
  TARGET vive_nodelet
  LIBS ${catkin_LIBRARIES} vive_driver config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_hw_msgs
)

# Launch files

install_launch_files()
