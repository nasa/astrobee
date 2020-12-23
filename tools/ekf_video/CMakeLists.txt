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
project(ekf_video)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ekf_bag
)

# System dependencies are found with CMake's conventions
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Gui)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(FFMPEG REQUIRED )

catkin_package(
  CATKIN_DEPENDS roscpp ekf_bag
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ekf_video
  src/ekf_bag_video.cc
  src/video_writer.cc
)
add_dependencies(ekf_video ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_video ${catkin_LIBRARIES} ${Qt5Gui_LIBRARIES} ${FFMPEG_LIBRARIES}
)


## Declare a C++ executable: inspection_tool
add_executable(ekf_to_video tools/ekf_to_video.cc)
add_dependencies(ekf_to_video ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_to_video
  ekf_video gflags glog ${catkin_LIBRARIES})