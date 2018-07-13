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

project(smart_dock)

set(DEPS
  rapidExtAstrobee
)

set(LIBS
  Qt4::QtXml
  rapidIo
  rapidExtAstrobee
  common)

set(INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SORACORE_INCLUDE_DIRS}
)

set(RTI_QOS_FILES
  ${CMAKE_SOURCE_DIR}/submodules/common/communications/dds/NDDS_DISCOVERY_PEERS
  ${CMAKE_SOURCE_DIR}/submodules/common/communications/dds/RAPID_QOS_PROFILES.xml
)

set(CONFIG_DIR ${CMAKE_SOURCE_DIR}/astrobee/config/communications/dds)

create_library(TARGET smart_dock
  LIBS ${LIBS} i2c eps_driver
  INC ${INCLUDES}
  DEPS ${DEPS}
)

create_tool_targets(DIR tools
  LIBS ${LIBS} smart_dock config_reader
  INC ${INCLUDES}
  DEPS ${DEPS}
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

# Copy these files outside of an install to make available
# with rosrun/roslaunch
file(MAKE_DIRECTORY ${CONFIG_DIR})
file(COPY ${RTI_QOS_FILES}
  DESTINATION ${CONFIG_DIR})

