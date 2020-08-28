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

project(graph_localizer)

# For gprof - remove otherwise
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# For google prof: 
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed -lprofiler -Wl,--as-needed")

find_package(GTSAM REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} camera config_reader ff_nodelet imu_integration localization_common localization_measurements msg_conversions
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} 
  CATKIN_DEPENDS roscpp 
  DEPENDS gtsam ff_msgs #gtsam_unstable  
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} gtsam camera config_reader ff_nodelet imu_integration localization_common localization_measurements msg_conversions #gtsam_unstable  
  INC ${catkin_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} 
  DEPS ff_msgs
)

install_launch_files()
