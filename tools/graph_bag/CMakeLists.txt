#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 2.8.3)

project(graph_bag)

# For gprof - remove otherwise
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# For google prof: 
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed -lprofiler -Wl,--as-needed")

catkin_package(
  CATKIN_DEPENDS roscpp rosbag
)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
create_library(TARGET ${PROJECT_NAME} 
  LIBS ff_common ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} camera ff_nodelet graph_localizer lk_optical_flow localization_measurements localization_node sparse_mapping msg_conversions 
  INC ${catkin_INCLUDE_DIRS}
)

create_tool_targets(DIR tools
  LIBS ${PROJECT_NAME} profiler
  INC ${catkin_INCLUDE_DIRS}
)
