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
project(optimization_common)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-DROS1)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
	localization_common
)

find_package(Eigen3 REQUIRED)

# Non-linear optimizer
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} ${CERES_LIBRARIES} 
  CATKIN_DEPENDS localization_common 
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/utilities.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES}) 

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_utilities_optimization_common
    test/test_utilities.test
    test/test_utilities.cc
  )
  target_link_libraries(test_utilities_optimization_common
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_se3_local_parameterization
    test/test_se3_local_parameterization.test
    test/test_se3_local_parameterization.cc
  )
  target_link_libraries(test_se3_local_parameterization
    ${PROJECT_NAME} ${catkin_LIBRARIES}   
  )
endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
