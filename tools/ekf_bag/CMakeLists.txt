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
project(ekf_bag)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  config_reader
  camera
  ekf
  lk_optical_flow
  localization_node
)


# Find OpenCV
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp config_reader camera ekf lk_optical_flow localization_node
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
add_library(ekf_bag
  src/ekf_bag.cc
  src/ekf_bag_csv.cc
  src/tracked_features.cc
)
add_dependencies(ekf_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(bag_to_csv tools/bag_to_csv.cc)
add_dependencies(bag_to_csv ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_to_csv
  ekf_bag gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(sparse_map_bag tools/sparse_map_bag.cc)
add_dependencies(sparse_map_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparse_map_bag
  ekf_bag gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install C++ executables
install(TARGETS bag_to_csv DESTINATION bin)
install(TARGETS bag_to_csv DESTINATION share/${PROJECT_NAME})
install(TARGETS sparse_map_bag DESTINATION bin)
install(TARGETS sparse_map_bag DESTINATION share/${PROJECT_NAME})