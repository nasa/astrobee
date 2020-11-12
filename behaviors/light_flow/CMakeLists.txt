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

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


# create_library(
#   DIR src/light_flow
#   TARGET light_flow
#   DEPS
#     ff_msgs
#     ff_hw_msgs
#   LIBS
#     ${catkin_LIBRARIES}
#     ff_nodelet
#     ${JSONCPP_LIBRARIES}
#   INC
#     ${catkin_INCLUDE_DIRS}
#     ${JSONCPP_INCLUDE_DIRS}
# )

# create_tool_targets(
#   DIR
#     tools
#   LIBS
#     ${catkin_LIBRARIES}
#     ff_nodelet
#     ${JSONCPP_LIBRARIES}
#     light_flow
#   INC
#     ${catkin_INCLUDE_DIRS}
#     ${JSONCPP_INCLUDE_DIRS}
#   DEPS
#     ff_msgs
#     ff_hw_msgs
#     light_flow
# )

# create_library(
#   DIR src/light_flow_nodelet
#   TARGET light_flow_nodelet
#   LIBS 
#     ${catkin_LIBRARIES} 
#     ${JSONCPP_LIBRARIES}
#     config_reader 
#     ff_nodelet 
#     light_flow
#   INC 
#     ${catkin_INCLUDE_DIRS}
#     ${JSONCPP_INCLUDE_DIRS}
#   DEPS 
#     ff_msgs
#     ff_hw_msgs
#     light_flow
# )

# install_launch_files()
