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

project(wdock)

set(DEPS
  rapidExtAstrobee
)

set(LIBS
  Qt4::QtXml
  rapidIo
  rapidExtAstrobee
  ff_common)

set(INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SORACORE_INCLUDE_DIRS}
)

create_library(TARGET wdock
  LIBS ${LIBS}
  INC ${INCLUDES}
  DEPS ${DEPS}
)

create_tool_targets(DIR tools
  LIBS ${LIBS} wdock
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
