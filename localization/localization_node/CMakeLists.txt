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
project(localization_node)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
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
# Find OpenCV and fix a 3.3.1 bug
find_package(OpenCV 3 REQUIRED)
if (${OpenCV_VERSION} MATCHES "3.3.1")
  foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
    set (__original_cvcomponent ${__cvcomponent})
    if(NOT __cvcomponent MATCHES "^opencv_")
      set(__cvcomponent opencv_${__cvcomponent})
    endif()
    if (TARGET ${__cvcomponent})
      set_target_properties(${__cvcomponent} PROPERTIES
          MAP_IMPORTED_CONFIG_DEBUG ""
          MAP_IMPORTED_CONFIG_RELEASE ""
          MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
          MAP_IMPORTED_CONFIG_MINSIZEREL ""
      )
    endif()
  endforeach(__cvcomponent)
endif()
set(OpenCV_LIBRARIES ${OpenCV_LIBS})

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


## Declare a C++ executable: inspection_tool
add_executable(extract_image_bag tools/extract_image_bag.cc)
add_dependencies(extract_image_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_image_bag
  localization_node gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(interest_points_test_bag tools/interest_points_test_bag.cc)
add_dependencies(interest_points_test_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(interest_points_test_bag
  localization_node gflags glog ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
