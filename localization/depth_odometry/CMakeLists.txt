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

project(depth_odometry)

if (USE_ROS)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
catkin_package(
  LIBRARIES ${PROJECT_NAME} ${PCL_LIBRARIES} ff_nodelet ${GTSAM_LIBRARIES} localization_common localization_measurements msg_conversions 
  INCLUDE_DIRS include ${GTSAM_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${PCL_CONVERSIONS_INCLUDE_DIRS}  
  CATKIN_DEPENDS roscpp pcl pcl_conversions 
  DEPENDS gtsam ff_msgs 
)

create_library(TARGET ${PROJECT_NAME} 
  LIBS ${catkin_LIBRARIES} gtsam ${PCL_LIBRARIES} ff_nodelet localization_common localization_measurements msg_conversions 
  INC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PCL_CONVERSIONS_INCLUDE_DIRS} 
  DEPS ff_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_point_to_plane
    test/test_point_to_plane.test
    test/test_point_to_plane.cc
  )
  target_link_libraries(test_point_to_plane
    depth_odometry gtsam
  )
endif()

install_launch_files()

endif (USE_ROS)
