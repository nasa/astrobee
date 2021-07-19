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
project(imu_integration)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	config_reader
	localization_common
	localization_measurements 
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES}
  INCLUDE_DIRS include ${GTSAM_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS} 
  CATKIN_DEPENDS config_reader localization_common localization_measurements 
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

# create_library(TARGET ${PROJECT_NAME} 
#   LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} config_reader localization_common localization_measurements 
#   INC ${catkin_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}  
#   DEPS gtsam 
# )
# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/dynamic_imu_filter.cc
  src/identity_filter.cc
  src/imu_filter.cc
  src/imu_integrator.cc
  src/latest_imu_integrator.cc
  src/utilities.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})