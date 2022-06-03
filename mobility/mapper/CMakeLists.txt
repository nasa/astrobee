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
project(mapper)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## This fixes the Eigen aligment issue
## http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D EIGEN_MAX_STATIC_ALIGN_BYTES=0")

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  ff_util
  ff_msgs
  message_runtime
  octomap
)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(Octomap REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
# Find PCL
find_package(PCL REQUIRED COMPONENTS common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    mapper
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    ff_msgs
    message_runtime
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(mapper
  src/callbacks.cc
  src/mapper_nodelet.cc
  src/octoclass.cc
  src/polynomials.cc
  src/sampled_trajectory.cc
  src/services.cc
  src/threads.cc
  src/visualization_functions.cc
)
add_dependencies(mapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapper ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Mapper initialization fault tester
  add_rostest_gtest(test_init_mapper
    test/test_init_mapper.test
    test/test_init_mapper.cc
  )

  target_link_libraries(test_init_mapper
    ${catkin_LIBRARIES} glog
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

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

