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
project(fam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_msgs
  ff_hw_msgs
  ff_util
  config_reader
  gnc_autocode
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES
    fam
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_msgs
    ff_hw_msgs
    ff_util
    config_reader
    gnc_autocode
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

# Declare C++ libraries
add_library(fam
  src/fam.cc
  src/fam_nodelet.cc
)
add_dependencies(fam ${catkin_EXPORTED_TARGETS})
target_link_libraries(fam ${catkin_LIBRARIES})

# create_library(TARGET fam
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} gnc_autocode msg_conversions ff_common config_reader ff_nodelet
#   INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
#   DEPS ff_msgs ff_hw_msgs)
#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


