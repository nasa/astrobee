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
project(ground_dds_ros_bridge)

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
find_package(Boost 1.54.0 QUIET REQUIRED COMPONENTS filesystem system iostreams
thread program_options timer)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")

if (USE_CTC)
  set(SORACORE_ROOT_DIR ${ARM_CHROOT_DIR}/user)
else (USE_CTC)
  set(SORACORE_ROOT_DIR /usr)
endif (USE_CTC)

set(MIRO_ROOT_DIR ${SORACORE_ROOT_DIR})

SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# find Qt version according to OS
find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --release OUTPUT_VARIABLE LSB_RELEASE_VERSION_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)

if (${LSB_RELEASE_VERSION_SHORT} VERSION_GREATER 18.04)
  find_package(Qt5Xml REQUIRED)
else ()
  find_package(Qt4 4.6.0 REQUIRED QtXml)
endif ()

find_package(Miro REQUIRED)
find_package(RtiDds REQUIRED)
find_package(Soracore REQUIRED)

catkin_package(
  LIBRARIES ground_dds_ros_bridge
  CATKIN_DEPENDS roscpp message_runtime std_msgs ff_msgs dds_msgs
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
add_library(ground_dds_ros_bridge
  ${cc_files}
)
target_compile_definitions(ground_dds_ros_bridge PUBLIC ${RTIDDS_DEFINE_FLAGS})
add_dependencies(ground_dds_ros_bridge ${catkin_EXPORTED_TARGETS})
if (${LSB_RELEASE_VERSION_SHORT} VERSION_GREATER 18.04)
  target_link_libraries(ground_dds_ros_bridge rapidIo Qt5:Xml ${Boost_LIBRARIES} ${catkin_LIBRARIES})
else ()
  target_link_libraries(ground_dds_ros_bridge rapidIo ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif ()

# Declare a C++ executable
add_executable(ground_dds_ros_bridge_node
  tools/ground_dds_ros_bridge_node.cc
)
add_dependencies(ground_dds_ros_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_dds_ros_bridge_node ground_dds_ros_bridge ${catkin_LIBRARIES})

else (USE_DDS)
  find_package(catkin REQUIRED COMPONENTS)
  catkin_package()
endif (USE_DDS)
