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

project(planner_trapezoidal)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    planner_trapezoidal
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    choreographer
    tf2_geometry_msgs
    ff_msgs
)

create_library(TARGET planner_trapezoidal
  LIBS ${catkin_LIBRARIES} ff_nodelet msg_conversions ff_flight config_server config_client jsonloader choreographer
  INC  ${catkin_INCLUDE_DIRS} 
  DEPS choreographer
)

install_launch_files()
