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
project(ff_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  trajectory_msgs
  sensor_msgs
)

# Generate messages in the 'msg' folder
file(GLOB MSG_FILES msg/*.msg)
foreach(SRC ${MSG_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND MSG_FILE_NAMES ${NAME})
endforeach()
add_message_files(FILES ${MSG_FILE_NAMES})

# Generate services in the 'srv' folder
file(GLOB SRV_FILES srv/*.srv)
foreach(SRC ${SRV_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND SRV_FILE_NAMES ${NAME})
endforeach()
add_service_files(FILES ${SRV_FILE_NAMES})

# Generate actions in the 'action' folder
file(GLOB ACTION_FILES action/*.action)
foreach(SRC ${ACTION_FILES})
  get_filename_component(NAME ${SRC} NAME)
  list(APPEND ACTION_FILE_NAMES ${NAME})
endforeach()
add_action_files(FILES ${ACTION_FILE_NAMES})

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs std_msgs
)

catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs actionlib_msgs trajectory_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
