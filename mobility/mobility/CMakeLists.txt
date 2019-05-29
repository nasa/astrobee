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

# Subsystem matapackage
project(mobility)

catkin_package(
  CATKIN_DEPENDS
    choreographer
    mapper
    sentinel
    planner_trapezoidal
    planner_qp
    docker
    ff_msgs
    ff_util
)

create_tool_targets(
  DIR tools
  LIBS ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} config_client common ff_flight planner_trapezoidal
  INC  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPS ff_msgs common
)

