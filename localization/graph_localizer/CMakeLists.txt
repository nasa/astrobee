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

if (USE_ROS)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} camera config_reader ff_nodelet graph_optimizer imu_integration localization_common localization_measurements msg_conversions
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} 
  CATKIN_DEPENDS roscpp 
  DEPENDS gtsam ff_msgs 
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} gtsam camera config_reader ff_nodelet graph_optimizer imu_integration localization_common localization_measurements msg_conversions 
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
  add_rostest_gtest(test_rotation_factor
    test/test_rotation_factor.test
    test/test_rotation_factor.cc
  )
  target_link_libraries(test_rotation_factor
    graph_localizer gtsam
  )
  add_rostest_gtest(test_point_to_line_factor
    test/test_point_to_line_factor.test
    test/test_point_to_line_factor.cc
  )
  target_link_libraries(test_point_to_line_factor
    graph_localizer gtsam
  )
  add_rostest_gtest(test_point_to_line_segment_factor
    test/test_point_to_line_segment_factor.test
    test/test_point_to_line_segment_factor.cc
  )
  target_link_libraries(test_point_to_line_segment_factor
    graph_localizer gtsam
  )
  add_rostest_gtest(test_point_to_plane_factor
    test/test_point_to_plane_factor.test
    test/test_point_to_plane_factor.cc
  )
  target_link_libraries(test_point_to_plane_factor
    graph_localizer localization_measurements gtsam
  )
  add_rostest_gtest(test_point_to_handrail_endpoint_factor
    test/test_point_to_handrail_endpoint_factor.test
    test/test_point_to_handrail_endpoint_factor.cc
  )
  target_link_libraries(test_point_to_handrail_endpoint_factor
    graph_localizer gtsam
  )
  add_rostest_gtest(test_silu
    test/test_silu.test
    test/test_silu.cc
  )
  target_link_libraries(test_silu
    graph_localizer gtsam
  )
endif()

install_launch_files()

endif (USE_ROS)
