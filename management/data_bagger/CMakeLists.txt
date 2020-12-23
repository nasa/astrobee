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

cmake_minimum_required(VERSION 2.8.12) # Need get_filename_component DIRECTORY
project(data_bagger)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ff_msgs
  nodelet
  ff_util
  config_reader
)

catkin_package(
  LIBRARIES data_bagger astrobee_recorder
  DEPENDS roscpp ff_msgs nodelet ff_util config_reader
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
add_library(data_bagger
  src/data_bagger.cc
)
add_dependencies(data_bagger ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_bagger ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(astrobee_recorder
  src/astrobee_recorder.cc
)
add_dependencies(astrobee_recorder ${catkin_EXPORTED_TARGETS})
target_link_libraries(astrobee_recorder ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/ DESTINATION config)
