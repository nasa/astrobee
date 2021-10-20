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
project(rviz_visualizer)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  ff_util
  config_reader
)
catkin_package(
  CATKIN_DEPENDS roscpp visualization_msgs ff_util config_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable: overhead_info
add_executable(overhead_info tools/overhead_info.cc)
add_dependencies(overhead_info ${catkin_EXPORTED_TARGETS})
target_link_libraries(overhead_info
  gflags ${catkin_LIBRARIES})

## Declare a C++ executable: visualization_node
add_executable(visualization_node tools/visualization_node.cc)
add_dependencies(visualization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualization_node
  gflags ${catkin_LIBRARIES})

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

#############
## Install ##
#############

# Install C++ executables
install(TARGETS overhead_info DESTINATION bin)
install(TARGETS visualization_node DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/overhead_info share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/visualization_node share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")
