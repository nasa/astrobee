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

#TODO(rsoussan): Is this needed here and in graph_bag?
if(USE_GOOGLE_PROFILER)
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed -lprofiler -Wl,--as-needed")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif()

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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_rotation_factor_adder
    test/test_rotation_factor_adder.test
    test/test_rotation_factor_adder.cc
  )
  target_link_libraries(test_rotation_factor_adder
    graph_localizer gtsam
  )
endif()


install_launch_files()
