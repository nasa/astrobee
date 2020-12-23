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
project(ekf_bag)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  config_reader
  camera
  ekf
  lk_optical_flow
  localization_node
)

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
  CATKIN_DEPENDS roscpp config_reader camera ekf lk_optical_flow localization_node
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
add_library(ekf_bag
  src/ekf_bag.cc
  src/ekf_bag_csv.cc
  src/tracked_features.cc
)
add_dependencies(ekf_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(bag_to_csv tools/bag_to_csv.cc)
add_dependencies(bag_to_csv ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_to_csv
  ekf_bag gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: bag_to_csv
add_executable(sparse_map_bag tools/sparse_map_bag.cc)
add_dependencies(sparse_map_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparse_map_bag
  ekf_bag gflags glog ${catkin_LIBRARIES})
