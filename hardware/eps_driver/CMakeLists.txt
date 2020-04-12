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

project(eps_driver)

catkin_package(
  LIBRARIES
    eps_driver
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    ff_hw_msgs
)

# ROS agnostic proxy library and tool

create_library(
  DIR src/eps_driver
  TARGET eps_driver
  LIBS i2c
)

create_tool_targets(
  DIR tools/eps_driver_tool
  LIBS eps_driver common
)

# ROS node

create_library(
  DIR src/eps_driver_node
  TARGET eps_driver_node
  LIBS ${catkin_LIBRARIES} eps_driver config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_hw_msgs
)

# Simulator

create_tool_targets(
  DIR tools/eps_simulator
  LIBS ${catkin_LIBRARIES} eps_driver ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_hw_msgs
)

install_launch_files()
install(FILES
  tools/eps_aux.py
  DESTINATION share/${PROJECT_NAME})

