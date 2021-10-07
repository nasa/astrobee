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
project(visualeyez)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  std_srvs
  ff_util
  config_reader
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(Ceres REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    tf2
    tf2_ros
    geometry_msgs
    visualization_msgs
    std_srvs
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
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(visualeyez
  src/visualeyez.cc
  src/visualeyez_bridge.cc
  src/visualeyez_server.cc
)
add_dependencies(visualeyez ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualeyez ${CERES_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ executable: visualeyez_tool
add_executable(visualeyez_tool tools/visualeyez_tool.cc)
add_dependencies(visualeyez_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualeyez_tool
  visualeyez gflags ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${catkin_LIBRARIES})

# Only test if it is enabled
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # We need to use the common client interface
  include_directories(${catkin_INCLUDE_DIRS} include)

  # A simple test to check that the streaming client receives, decodes 
  # and fails gracefully if a packet is malformed.
  add_rostest_gtest(test_visualeyez_stream
    test/test_visualeyez_stream.test
    test/test_visualeyez_stream.cc)
  target_link_libraries(test_visualeyez_stream visualeyez)

  # A simple test to check that the streaming client receives, decodes 
  # and fails gracefully if a packet is malformed.
  add_rostest_gtest(test_visualeyez_util
    test/test_visualeyez_util.test
    test/test_visualeyez_util.cc)
  target_link_libraries(test_visualeyez_util visualeyez)

endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install C++ executables
install(TARGETS visualeyez_tool DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/visualeyez_tool share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)