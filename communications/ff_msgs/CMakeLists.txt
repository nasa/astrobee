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

project(ff_msgs)

cmake_minimum_required(VERSION 3.5)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages in the 'msg' folder
file(GLOB MSG_FILES msg/*.msg)
foreach(SRC ${MSG_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND MSG_FILE_NAMES "msg/${NAME}")
endforeach()

# Generate services in the 'srv' folder
file(GLOB SRV_FILES srv/*.srv)
foreach(SRC ${SRV_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND SRV_FILE_NAMES "srv/${NAME}")
endforeach()

file(GLOB ACTION_FILES action/*.action)
foreach(SRC ${ACTION_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND ACTION_FILE_NAMES "action/${NAME}")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILE_NAMES}
  ${MSG_FILE_NAMES}
  ${SRV_FILE_NAMES}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  actionlib_msgs
  diagnostic_msgs
  tf2_geometry_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
