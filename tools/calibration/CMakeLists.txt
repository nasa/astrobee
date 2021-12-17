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
project(calibration)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  camera 
  localization_common
  msg_conversions
  optimization_common
  vision_common
)

# Find OpenCV3
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV 3 REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} 
  CATKIN_DEPENDS camera localization_common msg_conversions optimization_common vision_common 
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
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/parameter_reader.cc
  src/utilities.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

## Declare a C++ executable: create_undistorted_images
add_executable(create_undistorted_images tools/create_undistorted_images.cc)
add_dependencies(create_undistorted_images ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_undistorted_images
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES} )

## Declare a C++ executable: create_undistorted_images
add_executable(run_camera_target_based_intrinsics_calibrator tools/run_camera_target_based_intrinsics_calibrator.cc)
add_dependencies(run_camera_target_based_intrinsics_calibrator ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_camera_target_based_intrinsics_calibrator
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES})

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
install(TARGETS create_undistorted_images DESTINATION bin)
install(TARGETS run_camera_target_based_intrinsics_calibrator DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/create_undistorted_images share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/run_camera_target_based_intrinsics_calibrator share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_camera_target_based_intrinsics_calibrator
    test/test_camera_target_based_intrinsics_calibrator.test
    test/test_camera_target_based_intrinsics_calibrator.cc
    test/test_utilities.cc
  )
  target_link_libraries(test_camera_target_based_intrinsics_calibrator
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
endif()
