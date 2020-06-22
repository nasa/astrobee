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

project(ff_hw_msgs)

find_package(catkin2 REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
)

create_msg_targets(DIR msg SDIR srv DEPS std_msgs sensor_msgs)

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)