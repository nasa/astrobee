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
project(msg_conversions)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
  config_reader
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs config_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(msg_conversions
  src/msg_conversions.cc
)
add_dependencies(msg_conversions ${catkin_EXPORTED_TARGETS})
target_link_libraries(msg_conversions ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(landmark_msg_cnv tools/landmark_msg_cnv.cc)
add_dependencies(landmark_msg_cnv ${catkin_EXPORTED_TARGETS})
target_link_libraries(landmark_msg_cnv
  msg_conversions gflags ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(pose_stamped_msg_cnv tools/pose_stamped_msg_cnv.cc)
add_dependencies(pose_stamped_msg_cnv ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_stamped_msg_cnv
  msg_conversions gflags ${catkin_LIBRARIES})

# create_library(TARGET msg_conversions
#   LIBS ${catkin_LIBRARIES} config_reader
#   DEPS ff_msgs eigen
#   INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
# )

# create_tool_targets(DIR tools
#   LIBS ${catkin_LIBRARIES} ${GFLAGS_LIBRARIES} ff_common
#   INC ${catkin_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIRS}
#   DEPS ff_msgs
# )
