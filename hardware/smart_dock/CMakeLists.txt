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
project(smart_dock)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  i2c
  eps_driver
  dds_msgs
)

set(DEPS
  rapidExtAstrobee
)

set(LIBS
  Qt4::QtXml
  rapidIo
  rapidExtAstrobee
  ff_common)

set(INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SORACORE_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
    signal_lights
  CATKIN_DEPENDS
    roscpp
    nodelet
    i2c
    eps_driver
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
add_library(smart_dock
  src/smart_dock.cc
  src/smart_dock_node.cc
)
add_dependencies(smart_dock ${catkin_EXPORTED_TARGETS})
target_link_libraries(smart_dock ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(smart_dock_service tools/smart_dock_service.cc)
add_dependencies(smart_dock_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(smart_dock_service
  smart_dock gflags ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(smart_dock_tool tools/smart_dock_tool.cc)
add_dependencies(smart_dock_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(smart_dock_tool
  smart_dock gflags ${catkin_LIBRARIES})

# Determine our module name
get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

#############
## Install ##
#############

install(CODE "execute_process(
  COMMAND mkdir -p share/${MODULE_NAME}
  COMMAND ln -s ../../bin/dds_ros_bridge share/${MODULE_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")
