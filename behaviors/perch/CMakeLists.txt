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
project(perch)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ff_hw_msgs
  ff_msgs
  ff_util
)

## Find ament and libraries
find_package(ament_cmake REQUIRED)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

# Declare C++ libraries
add_library(perch SHARED
  src/perch_component.cc
)
target_compile_definitions(perch PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(perch ${dependencies})
rclcpp_components_register_nodes(perch "perch::PerchComponent")

## Declare a C++ executable: arm_tool
add_executable(perch_tool tools/perch_tool.cc)
ament_target_dependencies(perch_tool ${dependencies})
target_link_libraries(perch_tool
  perch gflags)

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Install C++ executables
install(TARGETS perch_tool DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../lib/perch_tool share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()