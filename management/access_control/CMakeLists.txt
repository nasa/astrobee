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

project(access_control)

catkin_package(
  LIBRARIES access_control
  DEPENDS roscpp ff_msgs nodelet
)

create_library(TARGET access_control
  LIBS ${catkin_LIBRARIES} config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_msgs config_reader
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_access_control 
    test/test_access_control.test
    test/test_access_control.cc
  )

  target_link_libraries(test_access_control
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
endif()

install_launch_files()
