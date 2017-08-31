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

set(LIBS
  config_reader
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
set(INCLUDES
  config_reader
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

create_library(TARGET camera
  LIBS ${LIBS}
  INC ${INCLUDES}
)

create_tool_targets(DIR tools
  LIBS camera config_reader
  INC ${INCLUDES}
)

create_test_targets(DIR test
  LIBS camera ${CERES_LIBRARIES}
  INC ${INCLUDES} ${CERES_INCLUDE_DIRS}
)

