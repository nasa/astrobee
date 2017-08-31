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

project(laser)

# Core library
create_library(TARGET laser
  INC ${GLOG_INCLUDE_DIRS}
  LIBS ${GLOG_LIBRARIES} i2c
  DEPS i2c)

# Standalone test programs
create_tool_targets(DIR tools
  INC ${GFLAGS_INCLUDE_DIRS}
  LIBS ${GFLAGS_LIBRARIES} laser common
  DEPS laser)

# Create ROS nodes if we are using ROS
if(USE_ROS)
  catkin_package(
    LIBRARIES laser
    CATKIN_DEPENDS roscpp roslib nodelet
    DEPENDS ff_hw_msgs
  )

  create_library(TARGET laser_nodelet
    DIR src/ros
    LIBS ${catkin_LIBRARIES} laser config_reader ff_nodelet
    INC ${catkin_INCLUDE_DIRS}
    DEPS ff_hw_msgs
  )
  install_launch_files()
endif(USE_ROS)
