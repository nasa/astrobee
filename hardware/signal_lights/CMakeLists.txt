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
project(signal_lights)

## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

set(dependencies
rclcpp
rclcpp_components
ff_util
ff_common
std_msgs
ff_msgs
ff_hw_msgs
config_reader
)

# Find ament and libraries
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
add_library(custom_i2c
  src/custom_i2c/i2c.cc
)
#add_dependencies(custom_i2c ${catkin_EXPORTED_TARGETS})
#target_link_libraries(custom_i2c ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(signal_lights
  src/signal_lights/signal_lights.cc
)
#add_dependencies(signal_lights ${catkin_EXPORTED_TARGETS})
target_link_libraries(signal_lights custom_i2c)

# Declare C++ libraries
add_library(signal_lights_nodelet SHARED
  src/signal_lights_node/signal_lights_nodelet.cc
)
target_compile_definitions(signal_lights_nodelet PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(signal_lights_nodelet signal_lights)
ament_target_dependencies(signal_lights_nodelet ${dependencies})
rclcpp_components_register_nodes(signal_lights_nodelet "signal_lights::SignalLightsNodelet")

## Declare a C++ executable: inspection_tool
add_executable(signal_lights_tool tools/signal_lights_tool.cc)
target_link_libraries(signal_lights_tool
  signal_lights gflags)
ament_target_dependencies(signal_lights_tool ff_common)

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS custom_i2c
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS signal_lights
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
#install(TARGETS signal_lights_nodelet
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)


# Install C++ executables
#install(TARGETS signal_lights_tool DESTINATION bin)
#install(CODE "execute_process(
#  COMMAND ln -s ../../bin/signal_lights_tool share/${PROJECT_NAME}
#  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
#  OUTPUT_QUIET
#  ERROR_QUIET
#  )")

# Mark launch files for installation
#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#  PATTERN ".svn" EXCLUDE)

ament_package()