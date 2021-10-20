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
project(config_reader)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_common
)

## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(Luajit20 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${LUAJIT_INCLUDE_DIR}
  LIBRARIES config_reader ${LUAJIT_LIBRARY} ${LUAJIT_MATH_LIBRARY}
  CATKIN_DEPENDS   roscpp nodelet ff_common
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LUAJIT_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(config_reader
  src/config_reader.cc
  src/watch_files.cc
)
add_dependencies(config_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(config_reader gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
