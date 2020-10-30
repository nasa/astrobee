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

project(cpu_mem_monitor)

catkin_package(
  LIBRARIES cpu_mem_monitor
  CATKIN_DEPENDS roscpp roslib nodelet ff_hw_msgs ff_msgs std_msgs
)

create_library(TARGET cpu_mem_monitor
  LIBS ${catkin_LIBRARIES} config_reader ff_nodelet
  INC ${catkin_INCLUDES}
  DEPS ff_msgs ff_hw_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # llp CPU monitor initialization fault tester
  add_rostest_gtest(test_init_llp_cpu_mem_monitor
    test/test_init_llp_cpu_mem_monitor.test
    test/test_init_llp_cpu_mem_monitor.cc
  )

  target_link_libraries(test_init_llp_cpu_mem_monitor
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
  # mlp CPU monitor initialization fault tester
  add_rostest_gtest(test_init_mlp_cpu_mem_monitor
    test/test_init_mlp_cpu_mem_monitor.test
    test/test_init_mlp_cpu_mem_monitor.cc
  )

  target_link_libraries(test_init_mlp_cpu_mem_monitor
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )

endif()

install_launch_files()
