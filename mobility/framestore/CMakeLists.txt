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
project(framestore)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

# Build for ROS2
add_definitions(-DROS2)

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(config_reader REQUIRED)
find_package(msg_conversions REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${ff_common_INCLUDE_DIRS}
  ${config_reader_INCLUDE_DIRS}
  ${msg_conversions_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(framestore SHARED
  src/framestore.cc
)
target_compile_definitions(framestore
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(framestore rclcpp rclcpp_components ff_util config_reader msg_conversions)
rclcpp_components_register_nodes(framestore "mobility::FrameStore")

## Declare a C++ executable: global_transforms
add_executable(global_transforms tools/global_transforms.cc)
target_link_libraries(global_transforms
  framestore gflags glog)
ament_target_dependencies(global_transforms rclcpp config_reader msg_conversions tf2_ros)

#############
## Install ##
#############

ament_export_include_directories(include)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install C++ executables
install(TARGETS global_transforms DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/global_transforms share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()