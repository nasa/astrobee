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
project(temp_monitor)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  i2c
  config_reader
  ff_hw_msgs
  ff_msgs
  ff_util
)

catkin_package(
  LIBRARIES
    temp_monitor
  CATKIN_DEPENDS
    roscpp
    nodelet
    i2c
    config_reader
    ff_util
    ff_hw_msgs
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
add_library(temp_monitor
  src/ADT7410.cc
  src/TCN75A.cc
  src/temp_monitor.cc
  src/temp_monitor_node.cc
)
add_dependencies(temp_monitor ${catkin_EXPORTED_TARGETS})
target_link_libraries(temp_monitor ${catkin_LIBRARIES})

## Declare a C++ executable: temp_monitor_tool
add_executable(temp_monitor_tool tools/temp_monitor_tool.cc)
add_dependencies(temp_monitor_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(temp_monitor_tool
  temp_monitor gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
