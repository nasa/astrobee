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

project(ff_util)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ff_nodelet config_server config_client perf_timer
  CATKIN_DEPENDS roscpp nodelet dynamic_reconfigure ff_msgs diagnostic_msgs tf2_geometry_msgs actionlib
)

create_library(TARGET ff_nodelet
  DIR src/ff_nodelet
  LIBS ${roscpp_LIBRARIES} ${nodelet_LIBRARIES} config_reader
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_msgs
)

create_library(TARGET ff_flight
  DIR src/ff_flight
  LIBS ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} config_reader msg_conversions
  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPS ff_msgs
)

create_library(TARGET config_server
  DIR src/config_server
  LIBS ${catkin_LIBRARIES} config_reader
  INC ${catkin_INCLUDE_DIRS}
  DEPS config_reader diagnostic_msgs
)

create_library(TARGET config_client
  DIR src/config_client
  LIBS ${catkin_LIBRARIES}
  INC ${catkin_INCLUDE_DIRS}
)

create_library(TARGET perf_timer
  DIR src/perf_timer
  LIBS ${catkin_LIBRARIES}
  INC ${catkin_INCLUDE_DIRS}
)

# Only test if it is enabled
if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(${catkin_INCLUDE_DIRS} include)

  # ff_action

  add_rostest_gtest(ff_action_nominal_behaviour
    test/ff_action_nominal_behaviour.test
    test/ff_action_nominal_behaviour.cc)
  target_link_libraries(ff_action_nominal_behaviour ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_active_timeout
    test/ff_action_active_timeout.test
    test/ff_action_active_timeout.cc)
  target_link_libraries(ff_action_active_timeout ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_cancel_own_goal
    test/ff_action_cancel_own_goal.test
    test/ff_action_cancel_own_goal.cc)
  target_link_libraries(ff_action_cancel_own_goal ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_connect_timeout
    test/ff_action_connect_timeout.test
    test/ff_action_connect_timeout.cc)
  target_link_libraries(ff_action_connect_timeout ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_deadline_timeout
    test/ff_action_deadline_timeout.test
    test/ff_action_deadline_timeout.cc)
  target_link_libraries(ff_action_deadline_timeout ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_preempt_others_goal
    test/ff_action_preempt_others_goal.test
    test/ff_action_preempt_others_goal.cc)
  target_link_libraries(ff_action_preempt_others_goal ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_preempt_own_goal
    test/ff_action_preempt_own_goal.test
    test/ff_action_preempt_own_goal.cc)
  target_link_libraries(ff_action_preempt_own_goal ff_nodelet ${catkin_LIBRARIES})

  add_rostest_gtest(ff_action_response_timeout
    test/ff_action_response_timeout.test
    test/ff_action_response_timeout.cc)
  target_link_libraries(ff_action_response_timeout ff_nodelet ${catkin_LIBRARIES})


endif()

install_launch_files()
