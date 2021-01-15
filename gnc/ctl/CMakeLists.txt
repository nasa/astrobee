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

project(ctl)

catkin_package(
 LIBRARIES ctl
 CATKIN_DEPENDS roscpp sensor_msgs nav_msgs
 DEPENDS ff_msgs ff_hw_msgs ff_common
)

create_library(TARGET ctl
  LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} gnc_autocode msg_conversions ff_common config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPS ff_msgs ff_hw_msgs)

if(CATKIN_ENABLE_TESTING and ENABLE_INTEGRATION_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_ctl test/test_ctl.test
    test/test_ctl_nominal.cc
  )
  target_link_libraries(test_ctl
    ctl
  )
endif()

install_launch_files()
