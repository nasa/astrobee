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

project(dds_ros_bridge)
catkin_package(
  LIBRARIES dds_ros_bridge astrobee_astrobee_bridge
  CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs
)

set(DEPS
  ff_msgs
)

set(LIBS
  ${Boost_IOSTREAMS_LIBRARY}
  config_reader
  ff_nodelet
  Qt5::Xml
  rapidIo
  rapidExtAstrobee
)

set(INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${SORACORE_INCLUDE_DIRS}
)

create_library(TARGET dds_ros_bridge
  LIBS ${LIBS}
  INC ${INCLUDES}
  DEPS ${DEPS}
)

create_library(TARGET astrobee_astrobee_bridge
  LIBS ${LIBS}
  INC ${INCLUDES}
  DEPS ${DEPS}
)

create_tool_targets(DIR tools
  LIBS dds_ros_bridge
  INC ${INCLUDES}
  DEPS ${DEPS} dds_ros_bridge
)

# Determine our module name
get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(CODE "execute_process(
  COMMAND mkdir -p share/${MODULE_NAME}
  COMMAND ln -s ../../bin/dds_ros_bridge share/${MODULE_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

install_launch_files()
