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

cmake_minimum_required(VERSION 2.8.3)
project(interest_point)

set(LIBS
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENMVG_LIBRARIES}
  ${CERES_LIBRARY}
  )
set(INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/common/include
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENMVG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interest_point
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

create_library(TARGET interest_point
  LIBS ${LIBS}
  INC ${INCLUDES}
)

set(INTEREST_LIBS
  interest_point
  ff_common
  ${LIBS})

create_tool_targets(DIR tools
  LIBS ${INTEREST_LIBS}
  INC ${INCLUDES}
)
create_test_targets(DIR test
  LIBS ${INTEREST_LIBS}
  INC ${INCLUDES}
)

set(INTEREST_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

