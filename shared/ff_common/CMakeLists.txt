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

project(ff_common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ff_common
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

create_library(TARGET ff_common
  INC ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include
  LIBS ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

create_test_targets(DIR test
  LIBS ff_common
  DEPS ff_common)
