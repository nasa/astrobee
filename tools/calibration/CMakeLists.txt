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

project(calibration)

if (USE_ROS)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${CERES_LIBRARIES} camera ff_nodelet localization_common msg_conversions optimization_common sparse_mapping 
  INCLUDE_DIRS include ${CERES_INCLUDE_DIRS} 
  CATKIN_DEPENDS roscpp rosbag
  DEPENDS 
)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ${CERES_LIBRARIES} camera ff_nodelet localization_common optimization_common sparse_mapping 
  INC ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} 
)

create_tool_targets(DIR tools
  LIBS ${PROJECT_NAME}
  INC ${catkin_INCLUDE_DIRS}
)

endif (USE_ROS)
