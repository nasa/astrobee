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

cmake_minimum_required(VERSION 2.8.3)
project(ff_util)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  config_reader
  roscpp
  nodelet
  dynamic_reconfigure
  ff_msgs diagnostic_msgs
  tf2_geometry_msgs
  actionlib
  msg_conversions
)

# Find jsoncpp
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(JsonCpp REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ff_nodelet config_server config_client perf_timer jsonloader
  CATKIN_DEPENDS config_reader roscpp nodelet dynamic_reconfigure ff_msgs diagnostic_msgs tf2_geometry_msgs actionlib msg_conversions
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ff_nodelet
  src/ff_nodelet/ff_nodelet.cc
)
add_dependencies(ff_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(ff_nodelet ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(ff_flight
  src/ff_flight/ff_flight.cc
)
add_dependencies(ff_flight ${catkin_EXPORTED_TARGETS})
target_link_libraries(ff_flight ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(config_server
  src/config_server/config_server.cc
)
add_dependencies(config_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(config_server ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(config_client
  src/config_client/config_client.cc
)
add_dependencies(config_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(config_client ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(perf_timer
  src/perf_timer/perf_timer.cc
)
add_dependencies(perf_timer ${catkin_EXPORTED_TARGETS})
target_link_libraries(perf_timer ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(jsonloader
  src/jsonloader/command.cc
  src/jsonloader/command_repo.cc
  src/jsonloader/keepout.cc
  src/jsonloader/keepoutio.cc
  src/jsonloader/plan.cc
  src/jsonloader/planio.cc
  src/jsonloader/validation.cc
)
add_dependencies(jsonloader ${catkin_EXPORTED_TARGETS})
target_link_libraries(jsonloader ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})


##########
## Test ##
##########

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

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/ DESTINATION config)
