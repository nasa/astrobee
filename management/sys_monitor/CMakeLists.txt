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

project(sys_monitor)

catkin_package(
  LIBRARIES sys_monitor
  DEPENDS roscpp ff_msgs nodelet
)

create_library(TARGET sys_monitor
  LIBS ${catkin_LIBRARIES} config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_msgs config_reader
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # System monitor initialization fault tester
  add_rostest_gtest(test_init_sys_monitor
    test/test_init_sys_monitor.test
    test/test_init_sys_monitor.cc
  )

  target_link_libraries(test_init_sys_monitor
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
  # System monitor tester
  add_rostest_gtest(test_sys_monitor
    test/test_sys_monitor.test
    test/test_sys_monitor.cc
  )

  target_link_libraries(test_sys_monitor
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
endif()

create_tool_targets(DIR tools
  LIBS sys_monitor
  DEPS sys_monitor
)

install_launch_files()
