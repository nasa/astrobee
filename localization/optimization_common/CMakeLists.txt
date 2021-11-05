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

project(optimization_common)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${CERES_LIBRARIES} localization_common
  INCLUDE_DIRS include ${CERES_INCLUDE_DIRS} 
  CATKIN_DEPENDS 
  DEPENDS 
)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ${CERES_LIBRARIES} localization_common 
  INC ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} 
  DEPS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_utilities
    test/test_utilities.test
    test/test_utilities.cc
  )
  target_link_libraries(test_utilities
    localization_common 
  )
endif()

#install_launch_files()
