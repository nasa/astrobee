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

cmake_minimum_required(VERSION 2.8.3)
project(vive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_util
  ff_hw_msgs
  config_reader
)

# System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)

# declare the catkin package
catkin_package(
  LIBRARIES
    vive_driver
    vive_nodelet
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_util
    ff_hw_msgs
    config_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(vive_driver
  src/vive_driver/vive.c
  src/vive_driver/vive_data_button.c
  src/vive_driver/vive_data_imu.c
  src/vive_driver/vive_data_light.c
  src/vive_driver/vive_data_ootx.c
  src/vive_driver/vive_dev_tracker.c
  src/vive_driver/vive_dev_watchman.c
  src/vive_driver/vive_usb.c
)
add_dependencies(vive_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_driver json-c usb-1.0 z ${catkin_LIBRARIES})

## Declare a C++ executable: vive_tool
add_executable(vive_tool tools/vive_tool/vive_tool.c)
add_dependencies(vive_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_tool
  vive_driver argtable2 ${catkin_LIBRARIES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Declare C++ libraries
add_library(vive_nodelet
  src/vive_nodelet/vive_nodelet.cc
)
add_dependencies(vive_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_nodelet vive_driver ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)