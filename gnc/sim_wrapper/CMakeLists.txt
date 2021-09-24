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
project(sim_wrapper)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_msgs
  ff_hw_msgs
  ff_util
  config_reader
  gnc_autocode
  camera
  sparse_mapping
)

catkin_package(
  LIBRARIES
    sim_wrapper
  CATKIN_DEPENDS
    roscpp
    ff_msgs
    ff_hw_msgs
    ff_util
    config_reader
    gnc_autocode
    camera
    sparse_mapping
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
add_library(sim_wrapper
  src/sim.cc
  src/sim_nodelet.cc
)
add_dependencies(sim_wrapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_wrapper ${catkin_LIBRARIES})

## Declare a C++ executable: sim_node
add_executable(sim_node tools/sim_node.cc)
add_dependencies(sim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_node
  sim_wrapper gflags ${catkin_LIBRARIES})

## Declare a C++ executable: sim_node
add_executable(test_sim tools/test_sim.cc)
add_dependencies(test_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_sim
  sim_wrapper ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install C++ executables
install(TARGETS sim_node DESTINATION bin)
install(TARGETS sim_node DESTINATION share/${PROJECT_NAME})
install(TARGETS test_sim DESTINATION bin)
install(TARGETS test_sim DESTINATION share/${PROJECT_NAME})

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
