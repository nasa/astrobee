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

project(pmc_actuator)

catkin_package(
  LIBRARIES
    pmc_actuator
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    std_msgs
    ff_msgs
    ff_hw_msgs
)

create_library(
  TARGET pmc_actuator
  LIBS ${catkin_LIBRARIES} i2c config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_hw_msgs
)

create_tool_targets(DIR tools
  LIBS ${catkin_LIBRARIES} pmc_actuator
  INC ${catkin_INCLUDE_DIRS}
)

install_launch_files()
install(FILES tools/nozzle_identification.csv DESTINATION share/${PROJECT_NAME})
