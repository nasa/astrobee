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

cmake_minimum_required(VERSION 2.8.3)

project(traj_opt_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Polynomial.msg
  SolverInfo.msg
  Spline.msg
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  LIBRARIES
)