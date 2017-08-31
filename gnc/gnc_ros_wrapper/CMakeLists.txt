# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

project(gnc_ros_wrapper)

catkin_package(
 LIBRARIES gnc_ros_wrapper
 CATKIN_DEPENDS roscpp sensor_msgs nav_msgs
 DEPENDS ff_msgs ff_hw_msgs common
)

create_library(TARGET gnc_ros_wrapper
  LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} gnc_autocode msg_conversions camera common config_reader ff_nodelet ff_flight
  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPS ff_msgs ff_hw_msgs)

create_tool_targets(DIR ros
  LIBS gnc_ros_wrapper
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_gnc_wrapper test/test_gnc_wrapper.test
    test/test_gnc_wrapper_nominal.cc
  )
  target_link_libraries(test_gnc_wrapper
    ${catkin_LIBRARIES}
    gnc_autocode
    msg_conversions
    camera
    common
    config_reader
    ff_nodelet
  )
endif()

install_launch_files()
