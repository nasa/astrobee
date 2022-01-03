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
project(dds_ros_bridge)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

if (USE_DDS)
## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  nodelet
  config_reader
  ff_util
  ff_msgs
  ff_hw_msgs
  dds_msgs
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost 1.54.0 QUIET REQUIRED COMPONENTS filesystem system iostreams thread program_options timer)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")

if (USE_CTC)
  set(SORACORE_ROOT_DIR ${ARM_CHROOT_DIR}/usr)
else (USE_CTC)
  set(SORACORE_ROOT_DIR /usr)
endif (USE_CTC)

set(MIRO_ROOT_DIR ${SORACORE_ROOT_DIR})

SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# just calls the normal one
find_package(Qt4 4.6.0 REQUIRED QtXml)
find_package(Miro REQUIRED)
find_package(RtiDds REQUIRED)
find_package(Soracore REQUIRED)


catkin_package(
  LIBRARIES dds_ros_bridge astrobee_astrobee_bridge
  CATKIN_DEPENDS nodelet config_reader ff_util ff_msgs ff_hw_msgs dds_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${RTIDDS_INCLUDE_DIR}
  ${SORACORE_INCLUDE_DIRS}
  ${MIRO_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}/Qt
  ${Boost_INCLUDE_DIRS}
)

file(GLOB cc_files
     "src/*.cc"
)

# Declare C++ libraries
add_library(dds_ros_bridge
  ${cc_files}
)
target_compile_definitions(dds_ros_bridge PUBLIC ${RTIDDS_DEFINE_FLAGS})
add_dependencies(dds_ros_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(dds_ros_bridge rapidIo ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(astrobee_astrobee_bridge
  src/astrobee_astrobee_bridge.cc
)
target_compile_definitions(astrobee_astrobee_bridge PUBLIC ${RTIDDS_DEFINE_FLAGS})
add_dependencies(astrobee_astrobee_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(astrobee_astrobee_bridge Qt4::QtXml rapidIo ${Boost_LIBRARIES} ${catkin_LIBRARIES})


#############
## Install ##
#############
# Determine our module name
get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(CODE "execute_process(
  COMMAND mkdir -p share/${MODULE_NAME}
  COMMAND ln -s ../../bin/dds_ros_bridge share/${MODULE_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark libraries for installation
install(TARGETS dds_ros_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS astrobee_astrobee_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
else (USE_DDS)
  find_package(catkin REQUIRED COMPONENTS)
  catkin_package()
endif (USE_DDS)