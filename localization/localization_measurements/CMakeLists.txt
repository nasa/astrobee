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

project(localization_measurements)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GTSAM_LIBRARIES} ff_common localization_common
  INCLUDE_DIRS include ${GTSAM_INCLUDE_DIR} 
  CATKIN_DEPENDS 
  DEPENDS gtsam ff_msgs 
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ff_common gtsam localization_common 
  INC ${catkin_INCLUDE_DIRS} 
  DEPS ff_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_plane
    test/test_plane.test
    test/test_plane.cc
  )
  target_link_libraries(test_plane
    localization_measurements gtsam
  )
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_measurement_conversions
    test/test_measurement_conversions.test
    test/test_measurement_conversions.cc
  )
  target_link_libraries(test_measurement_conversions
    localization_measurements gtsam
  )

endif()
