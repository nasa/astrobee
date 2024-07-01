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
project(factor_adders)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  graph_factors
	localization_common
  localization_measurements
  node_adders
  vision_common
)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS}
  CATKIN_DEPENDS graph_factors localization_common localization_measurements node_adders vision_common 
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Loc factor adder test
  add_rostest_gtest(test_loc_factor_adder
    test/test_loc_factor_adder.test
    test/test_loc_factor_adder.cc
  )
  target_link_libraries(test_loc_factor_adder
    ${catkin_LIBRARIES} 
  )
  # Single measurement based factor adder test
  add_rostest_gtest(test_single_measurement_based_factor_adder
    test/test_single_measurement_based_factor_adder.test
    test/test_single_measurement_based_factor_adder.cc
  )
  target_link_libraries(test_single_measurement_based_factor_adder
    ${catkin_LIBRARIES} 
  )
  # Standstill factor adder test
  add_rostest_gtest(test_standstill_factor_adder
    test/test_standstill_factor_adder.test
    test/test_standstill_factor_adder.cc
  )
  target_link_libraries(test_standstill_factor_adder
    ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_vo_smart_projection_factor_adder
    test/test_vo_smart_projection_factor_adder.test
    test/test_vo_smart_projection_factor_adder.cc
  )
  target_link_libraries(test_vo_smart_projection_factor_adder
    ${catkin_LIBRARIES} 
  )
endif()

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
