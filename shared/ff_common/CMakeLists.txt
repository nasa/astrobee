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

cmake_minimum_required(VERSION 2.8.3)
project(ff_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet
)

# ## System dependencies are found with CMake's conventions
find_package(Gflags REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ff_common
  CATKIN_DEPENDS
)

create_library(TARGET ff_common
  INC ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include
  LIBS ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_thread
    test/test_thread.test
    test/test_thread.cc
  )
  target_link_libraries(test_thread
    ff_common
  )

endif()

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ff_common
  src/init.cc
  src/thread.cc
  src/utils.cc
)
add_dependencies(ff_common ${catkin_EXPORTED_TARGETS} ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
target_link_libraries(ff_common gflags ${catkin_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

# create_test_targets(DIR test
#   LIBS ff_common
#   DEPS ff_common)
