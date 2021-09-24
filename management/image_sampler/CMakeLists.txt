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
project(image_sampler)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_msgs
  nodelet
  config_reader
  ff_util
  image_transport
)

# Find OpenCV3
find_package(OpenCV 3 REQUIRED)

catkin_package(
  LIBRARIES image_sampler
  DEPENDS roscpp ff_msgs nodelet config_reader ff_util image_transport
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# create_library(TARGET image_sampler
#   LIBS ${catkin_LIBRARIES} config_reader ff_nodelet ${OpenCV_LIBRARIES}
#   INC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
#   DEPS ff_msgs config_reader opencv
# )

# Declare C++ libraries
add_library(image_sampler
  src/image_sampler.cc
)
add_dependencies(image_sampler ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_sampler ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

