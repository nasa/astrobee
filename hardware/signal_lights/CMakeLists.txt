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

cmake_minimum_required(VERSION 3.0)
project(signal_lights)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-DROS1)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  ff_util
  std_msgs
  ff_msgs
  ff_hw_msgs
)

catkin_package(
  LIBRARIES
    signal_lights
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    std_msgs
    ff_msgs
    ff_hw_msgs
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
add_library(custom_i2c
  src/custom_i2c/i2c.cc
)
add_dependencies(custom_i2c ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_i2c ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(signal_lights
  src/signal_lights/signal_lights.cc
)
add_dependencies(signal_lights ${catkin_EXPORTED_TARGETS})
target_link_libraries(signal_lights custom_i2c ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(signal_lights_nodelet
  src/signal_lights_node/signal_lights_nodelet.cc
)
add_dependencies(signal_lights_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(signal_lights_nodelet signal_lights ${catkin_LIBRARIES})

## Declare a C++ executable: inspection_tool
add_executable(signal_lights_tool tools/signal_lights_tool.cc)
add_dependencies(signal_lights_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(signal_lights_tool
  signal_lights gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS custom_i2c
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS signal_lights
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS signal_lights_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install C++ executables
install(TARGETS signal_lights_tool DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/signal_lights_tool share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
