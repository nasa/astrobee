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
project(tutorial_examples)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  factor_adders
  graph_factors
  imu_integration
  localization_common
  localization_measurements
  node_adders
  sliding_window_graph_optimizer
  vision_common
)

# Find OpenCV
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV4WithXFeatures REQUIRED)


catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} 
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} 
  CATKIN_DEPENDS
    factor_adders
    graph_factors
    imu_integration
    localization_common
    localization_measurements
    node_adders
    sliding_window_graph_optimizer
    vision_common
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_simple_localizer
    test/test_simple_localizer.test
    test/test_simple_localizer.cc
  )
  target_link_libraries(test_simple_localizer
    ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_simple_odometry
    test/test_simple_odometry.test
    test/test_simple_odometry.cc
  )
  target_link_libraries(test_simple_odometry
    ${catkin_LIBRARIES} 
  )

endif() 
