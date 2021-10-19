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
project(dds_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

if (USE_DDS)
  ## Find catkin macros and libraries
  find_package(catkin REQUIRED)  # defines catkin_DIR
  SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
  find_package(catkin2 REQUIRED COMPONENTS
  )

  if (USE_CTC)
    set(SORACORE_ROOT_DIR ${ARM_CHROOT_DIR}/usr)
  else (USE_CTC)
    set(SORACORE_ROOT_DIR /usr)
  endif (USE_CTC)

  set(MIRO_ROOT_DIR ${SORACORE_ROOT_DIR})

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
  find_package(Qt4 4.6.0 REQUIRED QtXml)

  find_package(Miro REQUIRED)
  find_package(RtiDds REQUIRED)
  find_package(Soracore REQUIRED)

  catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/..
    LIBRARIES
      rapidExtAstrobee
  )

  ###########
  ## Build ##
  ###########
  # Specify additional locations of header files
  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${RTIDDS_INCLUDE_DIR}
    ${SORACORE_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/rapidDds
    ${SORACORE_IDL_DIR}/rapidDds
  )

  # Copying IDL files
  file(GLOB ASTROBEE_IDL_FILES
    "idl/*.idl")
  file(COPY ${ASTROBEE_IDL_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

  set(LIB_NAME "rapidExtAstrobee")

  include(CreateDdsMsgTargets)

  set(EXTRA_RTIDDS_IDL_ARGS -I${SORACORE_IDL_DIR}/rapidDds)
  create_dds_msg_targets(
    NAME ${LIB_NAME}
    DIR ${CMAKE_CURRENT_BINARY_DIR}
    INCLUDES "-I${SORACORE_IDL_DIR}/rapidDds")

  add_library(${LIB_NAME} ${rapidExtAstrobee_GENERATED})
  target_link_libraries(${LIB_NAME}
    rapidDds
    )
  target_compile_definitions(${LIB_NAME} PUBLIC ${RTIDDS_DEFINE_FLAGS})
  target_compile_options(${LIB_NAME} PUBLIC -Wno-write-strings)
  target_compile_options(${LIB_NAME} PRIVATE -Wno-strict-aliasing)
  target_include_directories(${LIB_NAME} PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${RTIDDS_INCLUDE_DIR}
    ${SORACORE_INCLUDE_DIRS}
    ${MIRO_INCLUDE_DIR})

  #############
  ## Install ##
  #############

  # Mark libraries for installation
  install(TARGETS ${LIB_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

  install(FILES ${rapidExtAstrobee_GENERATED_HEADERS}
          DESTINATION include/${LIB_NAME})

  # Mark cpp header files for installation
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
    FILES_MATCHING PATTERN "dds_msgs/*.h"
    PATTERN ".svn" EXCLUDE
  )
else (USE_DDS)
  find_package(catkin REQUIRED COMPONENTS)
  catkin_package()
endif (USE_DDS)