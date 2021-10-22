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
project(flashlight)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  roslib
  nodelet
  ff_hw_msgs
  i2c
  ff_util
  config_reader
)

catkin_package(
  LIBRARIES
    flashlight
  CATKIN_DEPENDS
    roscpp
    roslib
    nodelet
    ff_hw_msgs
    i2c
    ff_util
    config_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

  # Declare C++ librarie - Core library
  add_library(flashlight
    src/flashlight.cc
  )
  add_dependencies(flashlight ${catkin_EXPORTED_TARGETS})
  target_link_libraries(flashlight glog ${catkin_LIBRARIES})

  # Declare C++ librarie
  add_library(flashlight_nodelet
    src/ros/flashlight_nodelet.cc
  )
  add_dependencies(flashlight_nodelet ${catkin_EXPORTED_TARGETS})
  target_link_libraries(flashlight_nodelet flashlight ${catkin_LIBRARIES})

## Declare a C++ executable: flashlight_test
add_executable(flashlight_test tools/flashlight_test.cc)
add_dependencies(flashlight_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(flashlight_test
  flashlight gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install C++ executables
install(TARGETS flashlight_test DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/flashlight_test share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
