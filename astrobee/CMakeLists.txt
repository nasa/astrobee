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
project(astrobee)

set(ASTROBEE_VERSION 0.16.6-sem)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Verify the user has the pre-commit hook
execute_process(
  COMMAND cp ../scripts/git/pre-commit ../.git/hooks
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

#############
## Install ##
#############

install(DIRECTORY config/ DESTINATION config)
# resources will be installed separately
# install(DIRECTORY resources/ DESTINATION resources)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(PROGRAMS scripts/apk_print_version.sh
				 scripts/check_env.sh
                 scripts/print_version.sh
                 scripts/cpu_print_version.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install the version.txt file when install target is run
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.txt
  COMMAND ${CMAKE_COMMAND}
  -D OUTPUT_FILE=${CMAKE_CURRENT_BINARY_DIR}/version.txt
  -D ASTROBEE_VERSION=${ASTROBEE_VERSION}
  -D GIT_MODULE_FILE="${CMAKE_SOURCE_DIR}/../cmake/GetGitRevisionDescription.cmake"
  -P ${CMAKE_SOURCE_DIR}/../cmake/SetVersion.cmake)
add_custom_target(version DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/version.txt)
install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_CURRENT_BINARY_DIR}\" --target version)")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/version.txt DESTINATION ${CMAKE_INSTALL_PREFIX} )

install(PROGRAMS scripts/env_wrapper.sh DESTINATION ${CMAKE_INSTALL_PREFIX} )