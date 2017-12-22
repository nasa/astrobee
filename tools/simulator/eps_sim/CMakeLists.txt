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

project(eps_sim)

find_package(catkin2 REQUIRED COMPONENTS
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  AddRemoveBattery.srv
)

generate_messages(
  DEPENDENCIES sensor_msgs
)

#create_msg_targets(SDIR srv
#  DEPS sensor_msgs
#)

catkin_package(
  LIBRARIES eps_sim
  DEPENDS roscpp ff_msgs ff_nodelet
)

create_library(TARGET eps_sim
  LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} ff_nodelet
  INC ${catkin_INCLUDES} ${GLOG_INCLUDE_DIRS}
  DEPS ff_msgs eps_sim_generate_messages_cpp
)

install_launch_files()
