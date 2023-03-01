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
project(access_control)

## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS "${CMAKE_CXXX_FLAGS} -Wall -Werror -O3 -fPIC" )

## Find ament and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(config_reader REQUIRED)
find_package(ff_msgs REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

# Declare C++ libraries
add_library(access_control SHARED
  src/access_control.cc
)
target_compile_definitions(access_control PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(access_control rclcpp rclcpp_components ff_common ff_util config_reader ff_msgs)
rclcpp_components_register_nodes(access_control "access_control::AccessControl")

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  ament_add_gtest_executable(test_access_control 
    test/test_access_control.cc
  )
  target_link_libraries(test_access_control access_control)
  add_ros_test(test/test_access_control.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

#############
## Install ##
#############

ament_export_include_directories(include)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT access_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()