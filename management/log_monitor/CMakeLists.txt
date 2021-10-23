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

cmake_minimum_required(VERSION 3.0)
project(log_monitor)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  LIBRARIES
    log_monitor
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable: undistort_image
add_executable(log_tool tools/log_tool.cc)
add_dependencies(log_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(log_tool
  gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install C++ executables
install(TARGETS log_tool DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/log_tool share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")