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
project(dock)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ff_hw_msgs
  ff_msgs
  ff_util
)

catkin_package(
  LIBRARIES
    dock
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    ff_hw_msgs
    ff_msgs
    ff_util
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
add_library(dock
  src/dock_nodelet.cc
)
add_dependencies(dock ${catkin_EXPORTED_TARGETS})
target_link_libraries(dock ${catkin_LIBRARIES})

## Declare a C++ executable: dock_tool
add_executable(dock_tool tools/dock_tool.cc)
add_dependencies(dock_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(dock_tool
  dock gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install C++ executables
install(TARGETS dock_tool DESTINATION bin)
install(TARGETS dock_tool DESTINATION share/${PROJECT_NAME})

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
