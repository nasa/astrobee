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

project(graph_optimizer)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} config_reader localization_common localization_measurements msg_conversions
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} 
  CATKIN_DEPENDS 
  DEPENDS gtsam 
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} gtsam config_reader localization_common localization_measurements msg_conversions 
  INC ${catkin_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} 
  DEPS 
)
