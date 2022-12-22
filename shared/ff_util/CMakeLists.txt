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

cmake_minimum_required(VERSION 3.5)
project(ff_util)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(msg_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(actionlib REQUIRED)

set(CATKIN_PACKAGE_SHARE_DESTINATION "share/${PROJECT_NAME}")
set(CATKIN_PACKAGE_BIN_DESTINATION "lib/${PROJECT_NAME}")
set(CATKIN_PACKAGE_INCLUDE_DESTINATION "include/${PROJECT_NAME}")

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${msg_conversions_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ff_nodelet
  src/ff_nodelet/ff_nodelet.cc
)
target_link_libraries(ff_nodelet ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(ff_flight
  src/ff_flight/ff_flight.cc
)
target_link_libraries(ff_flight ${catkin_LIBRARIES})

ament_target_dependencies(ff_nodelet rclcpp ff_msgs diagnostic_msgs)
# ament_export_targets(ff_nodelet_export HAS_LIBRARY_TARGET)

ament_target_dependencies(ff_flight rclcpp geometry_msgs ff_msgs tf2_geometry_msgs)

# ament_target_dependencies(config_server rclcpp diagnostic_msgs)

# ament_target_dependencies(pose_stamped_msg_cnv rclcpp ff_common tf2_ros)

ament_export_include_directories(include ${config_reader_INCLUDE_DIRS})
ament_package()

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ff_timer
    test/test_ff_timer.test
    test/test_ff_timer.cc
  )
  target_link_libraries(test_ff_timer ff_nodelet)
  # ff_action
 # ament_add_gtest(ff_action_nominal_behaviour
 #   test/ff_action_nominal_behaviour.test
 #   test/ff_action_nominal_behaviour.cc)
 # target_link_libraries(ff_action_nominal_behaviour ff_nodelet ${catkin_LIBRARIES})

  # ff_action
#  ament_add_gtest(ff_action_nominal_behaviour
#    test/ff_action_nominal_behaviour.test
#    test/ff_action_nominal_behaviour.cc)
#  target_link_libraries(ff_action_nominal_behaviour ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_active_timeout
#    test/ff_action_active_timeout.test
#    test/ff_action_active_timeout.cc)
#  target_link_libraries(ff_action_active_timeout ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_cancel_own_goal
#    test/ff_action_cancel_own_goal.test
#    test/ff_action_cancel_own_goal.cc)
#  target_link_libraries(ff_action_cancel_own_goal ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_connect_timeout
#    test/ff_action_connect_timeout.test
#    test/ff_action_connect_timeout.cc)
#  target_link_libraries(ff_action_connect_timeout ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_deadline_timeout
#    test/ff_action_deadline_timeout.test
#    test/ff_action_deadline_timeout.cc)
#  target_link_libraries(ff_action_deadline_timeout ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_preempt_others_goal
#    test/ff_action_preempt_others_goal.test
#    test/ff_action_preempt_others_goal.cc)
#  target_link_libraries(ff_action_preempt_others_goal ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_preempt_own_goal
#    test/ff_action_preempt_own_goal.test
#    test/ff_action_preempt_own_goal.cc)
#  target_link_libraries(ff_action_preempt_own_goal ff_nodelet ${catkin_LIBRARIES})

#  add_rostest_gtest(ff_action_response_timeout
#    test/ff_action_response_timeout.test
#    test/ff_action_response_timeout.cc)
#  target_link_libraries(ff_action_response_timeout ff_nodelet ${catkin_LIBRARIES})

endif()

#############
## Install ##
#############

# Mark libraries for installation
# install(TARGETS ff_nodelet ff_flight config_server config_client perf_timer
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
