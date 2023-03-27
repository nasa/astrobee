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
project(light_flow)

## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

## Find ament macros and libraries
set(dependencies
  rclcpp
  rclcpp_components
  ff_hw_msgs
  ff_msgs
  ff_util
)

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
add_library(light_flow
  src/light_flow/light_flow.cc
)
ament_target_dependencies(light_flow rclcpp ff_hw_msgs ff_util ff_common)
target_link_libraries(light_flow jsoncpp)

# Declare C++ libraries
add_library(light_flow_component SHARED
  src/light_flow_component/light_flow_component.cc
)
target_compile_definitions(light_flow_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(light_flow_component light_flow jsoncpp)
ament_target_dependencies(light_flow_component ${dependencies})
rclcpp_components_register_nodes(light_flow_component "light_flow::LightFlowComponent")

## Declare a C++ executable: inspection_tool
add_executable(light_flow_tool tools/light_flow_tool.cc)
ament_target_dependencies(light_flow_tool ${dependencies})
target_link_libraries(light_flow_tool
  light_flow gflags jsoncpp )

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ${PROJECT_NAME}_component
  light_flow_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(CODE "execute_process(
  COMMAND ln -s ../../bin/light_flow_tool share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()