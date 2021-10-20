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
project(localization_node)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  nodelet
  camera
  ff_util
  sparse_mapping
  msg_conversions
)

# System dependencies are found with CMake's conventions
find_package(OpenMVG QUIET REQUIRED)
find_package(Eigen3 REQUIRED)

# Find OpenCV
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
  	message_runtime
  	std_msgs
  	sensor_msgs
  	cv_bridge
  	image_transport
  	rosbag
  	ff_msgs
  	tf2
  	sparse_mapping
    msg_conversions
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
add_library(localization_node
  src/localization.cc
  src/localization_nodelet.cc
)
add_dependencies(localization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENMVG_LIBRARIES}
)

## Declare a C++ executable: extract_image_bag
add_executable(extract_image_bag tools/extract_image_bag.cc)
add_dependencies(extract_image_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_image_bag
  localization_node gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: interest_points_test_bag
add_executable(interest_points_test_bag tools/interest_points_test_bag.cc)
add_dependencies(interest_points_test_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(interest_points_test_bag
  localization_node gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: merge_bags
add_executable(merge_bags tools/merge_bags.cc)
add_dependencies(merge_bags ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_bags
  localization_node gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install C++ executables
install(TARGETS extract_image_bag DESTINATION bin)
install(TARGETS interest_points_test_bag DESTINATION bin)
install(TARGETS merge_bags DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/extract_image_bag share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/interest_points_test_bag share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/merge_bags share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")


# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
