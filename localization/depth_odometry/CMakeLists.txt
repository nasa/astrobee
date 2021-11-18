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

cmake_minimum_required(VERSION 3.0)
project(depth_odometry)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  ff_util
  localization_common 
  localization_measurements 
  msg_conversions 
  optimization_common
  point_cloud_common
)

# Find OpenCV3
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV 3 REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common)

catkin_package(
  INCLUDE_DIRS include ${CERES_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${PCL_CONVERSIONS_INCLUDE_DIRS}  
  LIBRARIES ${PROJECT_NAME} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${GTSAM_LIBRARIES}   
  CATKIN_DEPENDS roscpp ff_msgs ff_util localization_common localization_measurements msg_conversions optimization_common point_cloud_common 
  DEPENDS gtsam pcl pcl_conversions  
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/brisk_feature_detector_and_matcher.cc
  src/depth_image_aligner.cc
  src/depth_odometry.cc
  src/depth_odometry_nodelet.cc
  src/depth_odometry_wrapper.cc
  src/good_features_to_track_detector.cc
  src/lk_optical_flow_feature_detector_and_matcher.cc
  src/parameter_reader.cc
  src/surf_feature_detector_and_matcher.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam)

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
