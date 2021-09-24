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
project(vive)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_util
  ff_hw_msgs
  config_reader
)

# System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)

# declare the catkin package
catkin_package(
  LIBRARIES
    vive_driver
    vive_nodelet
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_util
    ff_hw_msgs
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

# Declare C++ libraries
add_library(vive_driver
  src/vive_driver/vive.c
  src/vive_driver/vive_data_button.c
  src/vive_driver/vive_data_imu.c
  src/vive_driver/vive_data_light.c
  src/vive_driver/vive_data_ootx.c
  src/vive_driver/vive_dev_tracker.c
  src/vive_driver/vive_dev_watchman.c
  src/vive_driver/vive_usb.c
)
add_dependencies(vive_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_driver json-c usb-1.0 z ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(vive_nodelet
  src/vive_nodelet/vive_nodelet.cc
)
add_dependencies(vive_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_nodelet vive_driver ${catkin_LIBRARIES})

## Declare a C++ executable: vive_tool
add_executable(vive_tool tools/vive_tool/vive_tool.c)
add_dependencies(vive_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(vive_tool
  vive_driver argtable2 ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install C++ executables
install(TARGETS vive_tool DESTINATION bin)
install(TARGETS vive_tool DESTINATION share/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)