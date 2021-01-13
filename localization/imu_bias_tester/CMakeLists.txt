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

project(imu_bias_tester)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} ff_nodelet imu_integration localization_common localization_measurements
  INCLUDE_DIRS include ${GTSAM_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS} 
  CATKIN_DEPENDS 
  DEPENDS gtsam 
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} ff_nodelet imu_integration localization_common localization_measurements 
  INC ${catkin_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}  
  DEPS gtsam ff_msgs 
)

install_launch_files()
