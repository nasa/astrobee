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

project(description)

# Call catkin
catkin_package(
  CATKIN_DEPENDS
    roscpp roscpp tf2 tf2_ros geometry_msgs
)

# Create global_transforms
create_tool_targets(DIR tools
  LIBS ${catkin_LIBRARIES} common msg_conversions config_reader
  INC ${catkin_INCLUDE_DIRS}
  DEPS common msg_conversions config_reader tf2 tf2_ros geometry_msgs
)

# Create framestore
create_library(TARGET framestore
  LIBS ${catkin_LIBRARIES} common ff_nodelet msg_conversions
  INC  ${catkin_INCLUDE_DIRS}
  DEPS common msg_conversions roscpp nodelet pluginlib tf2 tf2_ros geometry_msgs
)

# Install our launch file
install_launch_files()

# Also install some other resources
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Check to see if correct version of media has been downloaded into tree. We
# dont do this if we are cross-compiling, as there is no need for it. We must
# also make sure the media is copied in a native install to a simulator.
if ((NOT USE_CTC) AND (NOT IS_BAMBOO_BUILD))
  ExternalProject_Add(media
    URL https://github.com/nasa/astrobee_media/archive/v0.0.1.tar.gz
    URL_HASH SHA1=5e6e35368273b4172110dfb3c651a04b4343e83c
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/media
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND "")
  add_dependencies(framestore media)
  install(DIRECTORY media DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif ((NOT USE_CTC) AND (NOT IS_BAMBOO_BUILD))
