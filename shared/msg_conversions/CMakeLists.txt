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

cmake_minimum_required(VERSION 3.5)
project(msg_conversions)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

find_package(Eigen3 REQUIRED)

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(config_reader REQUIRED)
find_package(ff_common REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
)

# Declare C++ libraries
add_library(msg_conversions
  src/msg_conversions.cc
)
ament_target_dependencies(msg_conversions rclcpp geometry_msgs tf2_geometry_msgs ff_msgs ff_common config_reader)
ament_export_targets(msg_conversions HAS_LIBRARY_TARGET)

## Declare a C++ executable: landmark_msg_cnv
add_executable(landmark_msg_cnv tools/landmark_msg_cnv.cc)
target_link_libraries(landmark_msg_cnv
  msg_conversions gflags)
ament_target_dependencies(landmark_msg_cnv rclcpp ff_msgs ff_common)

## Declare a C++ executable: pose_stamped_msg_cnv
add_executable(pose_stamped_msg_cnv tools/pose_stamped_msg_cnv.cc)
target_link_libraries(pose_stamped_msg_cnv
  msg_conversions gflags )
ament_target_dependencies(pose_stamped_msg_cnv rclcpp ff_common tf2_ros)

#############
## Install ##
#############

ament_export_include_directories(include ${config_reader_INCLUDE_DIRS})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT msg_conversions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install C++ executables
install(TARGETS landmark_msg_cnv DESTINATION bin)
install(TARGETS pose_stamped_msg_cnv DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/landmark_msg_cnv share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/pose_stamped_msg_cnv share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

ament_package()
