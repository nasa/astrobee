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
project(light_flow)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_hw_msgs
  ff_msgs
  nodelet
  ff_util
)

catkin_package(
  LIBRARIES
    light_flow
  CATKIN_DEPENDS
    roscpp
    ff_hw_msgs
    ff_msgs
    ff_util
    nodelet
    pluginlib
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
add_library(light_flow
  src/light_flow/light_flow.cc
)
add_dependencies(light_flow ${catkin_EXPORTED_TARGETS})
target_link_libraries(light_flow ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(light_flow_nodelet
  src/light_flow_nodelet/light_flow_nodelet.cc
)
add_dependencies(light_flow_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(light_flow light_flow_nodelet ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(light_flow_tool tools/light_flow_tool.cc)
add_dependencies(light_flow_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(light_flow_tool
  light_flow gflags jsoncpp ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install C++ executables
install(TARGETS light_flow_tool DESTINATION bin)
install(TARGETS light_flow_tool DESTINATION share/${PROJECT_NAME})

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
