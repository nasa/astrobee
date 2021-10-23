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
project(mobility)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  choreographer
  mapper
  planner_trapezoidal
  planner_qp
  ff_msgs
  ff_util
  jsonloader
)
catkin_package(
  CATKIN_DEPENDS
    choreographer
    mapper
    planner_trapezoidal
    planner_qp
    ff_msgs
    ff_util
    jsonloader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable: plangen
add_executable(plangen tools/plangen.cc)
add_dependencies(plangen ${catkin_EXPORTED_TARGETS})
target_link_libraries(plangen
  gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: teleop
add_executable(teleop tools/teleop.cc)
add_dependencies(teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop
  gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install C++ executables
install(TARGETS plangen DESTINATION bin)
install(TARGETS teleop DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/plangen share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/teleop share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")
