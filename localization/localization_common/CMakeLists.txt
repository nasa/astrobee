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

project(localization_common)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GTSAM_LIBRARIES} camera config_reader msg_conversions 
  INCLUDE_DIRS include ${GTSAM_INCLUDE_DIR} ${Boost_INCLUDE_DIRS}  
  CATKIN_DEPENDS 
  DEPENDS gtsam 
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} camera config_reader gtsam msg_conversions 
  INC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}   
  DEPS 
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_utilities_localization_common
    test/test_utilities.test
    test/test_utilities.cc
  )
  target_link_libraries(test_utilities_localization_common
    localization_common 
  )
endif()
