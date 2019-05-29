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

project(executive)

catkin_package(
  LIBRARIES
    executive
    sequencer
  CATKIN_DEPENDS
    roscpp
    ff_flight
    ff_msgs
    ff_util
    nodelet
)

create_library(TARGET sequencer
  DIR src/utils/sequencer/
  LIBS ${catkin_LIBRARIES} ${Boost_IOSTREAMS_LIBRARY} jsonloader msg_conversions
  INC ${catkin_INCLUDE_DIRS} msg_conversions
  DEPS ff_msgs jsonloader msg_conversions
)

create_library(TARGET executive
  LIBS ${catkin_LIBRARIES} ff_flight ff_nodelet config_reader sequencer config_client
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_flight ff_msgs ff_util config_reader sequencer
)

create_tool_targets(DIR tools
  LIBS executive ${catkin_LIBRARIES} ${Boost_IOSTREAMS_LIBRARY}
  INC ${catkin_INCLUDE_DIRS}
  DEPS executive
)

install_launch_files()
