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

project(config_reader)

cmake_minimum_required(VERSION 3.5)

# Default to C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ff_common REQUIRED)

## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(Luajit20 REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${LUAJIT_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(config_reader
  src/config_reader.cc
  src/watch_files.cc
)

ament_target_dependencies(config_reader rclcpp ff_common)
target_link_libraries(config_reader gflags glog ${LUAJIT_LIBRARY})
ament_export_targets(config_reader HAS_LIBRARY_TARGET)

#############
## Install ##
#############

ament_export_include_directories(include ${LUAJIT_INCLUDE_DIR})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT config_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
