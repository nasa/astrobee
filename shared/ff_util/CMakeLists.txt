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

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(config_reader REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(msg_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(config_reader REQUIRED)
# find_package(actionlib REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${config_reader_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ff_component
  src/ff_component/ff_component.cc
)
ament_target_dependencies(ff_component rclcpp ff_common ff_msgs diagnostic_msgs config_reader)
ament_export_targets(ff_component HAS_LIBRARY_TARGET)

# Declare C++ libraries
add_library(ff_flight
  src/ff_flight/ff_flight.cc
)
ament_target_dependencies(ff_flight rclcpp geometry_msgs ff_msgs ff_common msg_conversions config_reader tf2_geometry_msgs)

# ament_target_dependencies(config_server rclcpp diagnostic_msgs)

# ament_target_dependencies(pose_stamped_msg_cnv rclcpp ff_common tf2_ros)


#############
## Linting ##
#############

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ff_timer
  ament_add_gtest(test_ff_timer
    test/test_ff_timer.cc
  )
  target_link_libraries(test_ff_timer ff_component)

  # ff_service
  ament_add_gtest(test_ff_service
    test/test_ff_service.cc
  )
  target_link_libraries(test_ff_service ff_component)

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

ament_export_dependencies(
  config_reader
  msg_conversions
)

ament_export_include_directories(include)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark libraries for installation
install(TARGETS ff_component ff_flight #config_server config_client perf_timer
  EXPORT ff_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()