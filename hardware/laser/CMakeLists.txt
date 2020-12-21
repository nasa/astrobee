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
project(laser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nodelet
  cv_bridge
  ff_util
  ff_common
  ff_hw_msgs
  i2c
  config_reader
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser
  CATKIN_DEPENDS
    roscpp
    roslib
    nodelet
    cv_bridge
    ff_util
    ff_common
    ff_hw_msgs
    i2c
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
)

# Declare C++ libraries
add_library(laser
  src/laser.cc
)
add_dependencies(laser ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

# Declare C++ libraries
add_library(laser_nodelet
  src/ros/laser_nodelet.cc
)
add_dependencies(laser_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_nodelet ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(laser_test tools/laser_test.cc)
add_dependencies(laser_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_test
  laser gflags glog ${catkin_LIBRARIES})


# # Core library
# create_library(TARGET laser
#   INC ${GLOG_INCLUDE_DIRS}
#   LIBS ${GLOG_LIBRARIES} i2c
#   DEPS i2c)

# # Standalone test programs
# create_tool_targets(DIR tools
#   INC ${GFLAGS_INCLUDE_DIRS}
#   LIBS ${GFLAGS_LIBRARIES} laser ff_common
#   DEPS laser)


# create_library(TARGET laser_nodelet
#   DIR src/ros
#   LIBS ${catkin_LIBRARIES} laser config_reader ff_nodelet
#   INC ${catkin_INCLUDE_DIRS}
#   DEPS ff_hw_msgs
# )

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
