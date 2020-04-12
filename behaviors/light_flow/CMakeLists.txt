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

project(light_flow)

catkin_package(
  LIBRARIES
    light_flow
  CATKIN_DEPENDS
    roscpp
    ff_hw_msgs
    ff_msgs
    ff_util
    signal_lights
    nodelet
    pluginlib
)

create_library(
  DIR src/light_flow
  TARGET light_flow
  DEPS
    ff_msgs
    ff_hw_msgs
    signal_lights
  LIBS
    ${catkin_LIBRARIES}
    ff_nodelet
    signal_lights
    ${JSONCPP_LIBRARIES}
  INC
    ${catkin_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

create_tool_targets(
  DIR
    tools
  LIBS
    ${catkin_LIBRARIES}
    ff_nodelet
    ${JSONCPP_LIBRARIES}
    signal_lights
    light_flow
  INC
    ${catkin_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
  DEPS
    ff_msgs
    ff_hw_msgs
    signal_lights
    light_flow
)

create_library(
  DIR src/light_flow_nodelet
  TARGET light_flow_nodelet
  LIBS 
    ${catkin_LIBRARIES} 
    ${JSONCPP_LIBRARIES}
    signal_lights 
    config_reader 
    ff_nodelet 
    light_flow
  INC 
    ${catkin_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
  DEPS 
    ff_msgs
    ff_hw_msgs
    light_flow
    signal_lights
)

install_launch_files()