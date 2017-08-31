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

project(sim_wrapper)

catkin_package(
  LIBRARIES sim_wrapper
  CATKIN_DEPENDS roscpp geometry_msgs ff_util nodelet pluginlib
)

create_library(TARGET sim_wrapper
  LIBS ${catkin_LIBRARIES} gnc_autocode sparse_mapping msg_conversions camera ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_hw_msgs
)

create_tool_targets(DIR tools
  LIBS sim_wrapper
)

install_launch_files()