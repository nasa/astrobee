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
project(camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  rosbag
  ff_common
  ff_msgs
  nodelet
  config_reader
)

# System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    rosbag
    ff_common
    ff_msgs
    nodelet
    config_reader
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
add_library(camera
  src/camera_model.cc
  src/camera_params.cc
)
add_dependencies(camera ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})

## Declare a C++ executable: undistort_image
add_executable(undistort_image tools/undistort_image.cc)
add_dependencies(undistort_image ${catkin_EXPORTED_TARGETS})
target_link_libraries(undistort_image
  camera gflags glog ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_camera_model
    test/test_camera_model.test
    test/test_camera_model.cc
  )
  target_link_libraries(test_camera_model
    camera
  )

  add_rostest_gtest(test_camera_params
    test/test_camera_params.test
    test/test_camera_params.cc
  )
  target_link_libraries(test_camera_params
    camera
  )

endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install C++ executables
install(TARGETS undistort_image DESTINATION bin)
install(TARGETS undistort_image DESTINATION share/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)