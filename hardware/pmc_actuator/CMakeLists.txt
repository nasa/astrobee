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
project(pmc_actuator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  ff_util
  std_msgs
  ff_msgs
  ff_hw_msgs
  i2c
)

catkin_package(
  LIBRARIES
    pmc_actuator
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    std_msgs
    ff_msgs
    ff_hw_msgs
    i2c
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ROS agnostic driver and tool

# Declare C++ libraries
add_library(pmc_actuator
  src/pmc_actuator/pmc_actuator.cc
)
add_dependencies(pmc_actuator ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmc_actuator ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(pmc_actuator_tool tools/pmc_actuator_tool/pmc_actuator_tool.cc)
add_dependencies(pmc_actuator_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmc_actuator_tool
  pmc_actuator gflags glog ${catkin_LIBRARIES})

# ROS nodelet and tools

# Declare C++ libraries
add_library(pmc_actuator_nodelet
  src/pmc_actuator_nodelet/pmc_actuator_nodelet.cc
)
add_dependencies(pmc_actuator_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmc_actuator_nodelet ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(pmc_actuator_cmd_test tools/pmc_actuator_cmd_test.cc)
add_dependencies(pmc_actuator_cmd_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmc_actuator_cmd_test
  pmc_actuator gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(pmc_actuator_feeder tools/pmc_actuator_feeder.cc)
add_dependencies(pmc_actuator_feeder ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmc_actuator_feeder
  pmc_actuator gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(pmc_autotrim_node tools/pmc_autotrim_node.cc)
add_dependencies(pmc_autotrim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pmc_autotrim_node
  pmc_actuator gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(FILES
  tools/pmc_currents.py
  tools/pmc_status.py
  tools/emi_smooth_lowfreq.csv
  tools/emi_smooth_profile.csv
  tools/nozzle_identification.csv
  tools/nozzles_closed.csv
  tools/nozzles_medium.csv
  tools/nozzles_open.csv
  tools/nozzle_calibration.csv
  tools/impeller_left.csv
  tools/impeller_right.csv
  tools/prop_nominal.csv
  tools/prop_stress.csv
  DESTINATION share/${PROJECT_NAME})
