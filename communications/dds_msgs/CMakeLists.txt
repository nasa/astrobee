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
# if (USE_DDS)
cmake_minimum_required(VERSION 2.8.3)
project(dds_msgs)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  LIBRARIES
    rapidExtAstrobee
)

###########
## Build ##
###########

message(STATUS "Copying IDL files")
file(GLOB DDS_IDL_FILES
  "${CMAKE_SOURCE_DIR}/communications/dds_msgs/idl/*.idl")

file(COPY ${DDS_IDL_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set(LIB_NAME "rapidExtAstrobee")

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
  ${SORACORE_INCLUDE_DIRS})

#############
## Install ##
#############

install(TARGETS ${LIB_NAME} DESTINATION lib)
install(FILES ${rapidExtAstrobee_GENERATED_HEADERS}
        DESTINATION include/${LIB_NAME})

# endif (USE_DDS)