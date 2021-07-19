#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 2.8.12)
project(graph_optimizer)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	config_reader
	localization_common
	localization_measurements
	msg_conversions 
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES}
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS}
  CATKIN_DEPENDS config_reader localization_common localization_measurements msg_conversions
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/graph_optimizer.cc
  src/graph_values.cc
  src/utilities.cc
  src/graph_stats.cc
  src/parameter_reader.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
