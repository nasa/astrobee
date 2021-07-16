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
# if (USE_DDS)
cmake_minimum_required(VERSION 2.8.3)
project(dds_ros_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  config_reader
  ff_util
  ff_msgs
  dds_msgs
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# without this we find the wrong one in cross compile
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
find_package(Qt4 4.6.0 REQUIRED QtXml)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../dds_msgs/cmake")
find_package(Miro REQUIRED )
find_package(RtiDds REQUIRED )
find_package(Soracore REQUIRED )

catkin_package(
  LIBRARIES dds_ros_bridge
  CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs dds_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${RTIDDS_INCLUDE_DIR}
  ${SORACORE_INCLUDE_DIRS}
  ${MIRO_INCLUDE_DIR}
)

file(GLOB cc_files
     "src/util.cc"
)
add_library(dds_ros_bridge
  ${cc_files}
)
add_dependencies(dds_ros_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(dds_ros_bridge Qt4::QtXml rapidExtAstrobee ${RTIDDS_LIBRARIES} ${catkin_LIBRARIES})

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

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
# endif (USE_DDS)