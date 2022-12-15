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

cmake_minimum_required(VERSION 3.0)
project(gnc_visualizer)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_definitions(-DROS1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ff_msgs
)

catkin_package(
 LIBRARIES
 DEPENDS rospy std_msgs ff_msgs
)

