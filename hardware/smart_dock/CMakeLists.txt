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
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  i2c
  eps_driver
  dds_msgs
)

# System dependencies are found with CMake's conventions
find_package(Boost 1.54.0 QUIET REQUIRED COMPONENTS filesystem system iostreams thread program_options timer)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../communications/dds_msgs/cmake")

if (USE_CTC)
  set(SORACORE_ROOT_DIR ${ARM_CHROOT_DIR}/usr)
else (USE_CTC)
  set(SORACORE_ROOT_DIR /usr)
endif (USE_CTC)

set(MIRO_ROOT_DIR ${SORACORE_ROOT_DIR})

SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# just calls the normal one
find_package(Qt4 4.6.0 REQUIRED QtXml)
find_package(Miro REQUIRED)
find_package(RtiDds REQUIRED)
find_package(Soracore REQUIRED)
catkin_package(
  LIBRARIES
    signal_lights
  CATKIN_DEPENDS
    roscpp
    nodelet
    i2c
    eps_driver
    dds_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RTIDDS_INCLUDE_DIR}
  ${SORACORE_INCLUDE_DIRS}
  ${MIRO_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}/Qt
)


# Declare C++ libraries
add_library(smart_dock
  src/smart_dock.cc
  src/smart_dock_node.cc
)
target_compile_definitions(smart_dock PUBLIC ${RTIDDS_DEFINE_FLAGS})
add_dependencies(smart_dock ${catkin_EXPORTED_TARGETS})
target_link_libraries(smart_dock rapidIo ${catkin_LIBRARIES})

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
