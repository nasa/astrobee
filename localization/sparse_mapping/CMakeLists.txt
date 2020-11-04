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

cmake_minimum_required(VERSION 2.8.12) # Need get_filename_component DIRECTORY
project(sparse_mapping)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS protobuf/sparse_map.proto)
get_filename_component(PROTO_HDR_DIR ${PROTO_HDRS} DIRECTORY)

set(LIBS
  camera
  ${DBOW2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENMVG_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  interest_point
  config_reader
  ff_common
)
set(INCLUDES
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${NGRAPH_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENMVG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${PROTO_HDR_DIR}
  ${DBOW2_INCLUDE_DIRS}
)
set(SPARSE_MAPPING_LIBRARIES sparse_mapping ${LIBS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sparse_mapping
  CATKIN_DEPENDS camera interest_point config_reader ff_common
)

create_library(TARGET sparse_mapping
  LIBS ${LIBS}
  INC include ${INCLUDES}
  DEPS ${DEPS}
  ADD_SRCS ${PROTO_SRCS} )

if (NOT USE_CTC)
  create_tool_targets(DIR tools
    LIBS ${LIBS} sparse_mapping
    INC ${INCLUDES}
    DEPS ${DEPS}
    EXCLUDE ${EXCLUDE})
endif (NOT USE_CTC)

create_test_targets(DIR test
  LIBS ${LIBS} sparse_mapping
  INC ${INCLUDES}
  DEPS ${DEPS})

set(SPARSE_MAPPING_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${INTEREST_INCLUDE_DIRS}
  ${CAMERA_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

set(SPARSE_MAPPING_LIBRARIES ${SPARSE_MAPPING_LIBRARIES} PARENT_SCOPE)
set(SPARSE_MAPPING_INCLUDE_DIRS ${SPARSE_MAPPING_INCLUDE_DIRS} PARENT_SCOPE)
