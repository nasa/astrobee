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
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

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
  ${JSONCPP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(ff_util
  src/ff_component/ff_component.cc
  src/ff_flight/ff_flight.cc
  src/config_server/config_server.cc
  src/config_client/config_client.cc
)
ament_target_dependencies(ff_util rclcpp ff_common ff_msgs geometry_msgs msg_conversions diagnostic_msgs config_reader tf2_geometry_msgs)

ament_export_targets(ff_util HAS_LIBRARY_TARGET)

# ament_target_dependencies(pose_stamped_msg_cnv rclcpp ff_common tf2_ros)

add_library(test_ff_service SHARED
  test/test_ff_service_server.cc
  src/ff_component/ff_component.cc
)
target_compile_definitions(test_ff_service PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(test_ff_service rclcpp rclcpp_components ff_common config_reader ff_msgs)
rclcpp_components_register_nodes(test_ff_service "ff_util::TestServiceServer")

add_library(test_ff_client SHARED
  test/test_ff_service_client.cc
  src/ff_component/ff_component.cc
)
target_compile_definitions(test_ff_client PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(test_ff_client rclcpp rclcpp_components ff_common config_reader ff_msgs)
rclcpp_components_register_nodes(test_ff_client "ff_util::TestServiceClient")

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
  find_package(ros_testing REQUIRED)

  # ff_timer
  ament_add_gtest_executable(test_ff_timer
    test/test_ff_timer.cc
  )
  target_link_libraries(test_ff_timer ff_util)
  add_ros_test(test/launch/test_ff_timer.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # ff_service
  ament_add_gtest_executable(test_ff_service_nominal
    test/test_ff_service_nominal.cc
  )
  target_link_libraries(test_ff_service_nominal test_ff_service)
  add_ros_test(test/launch/test_ff_service_nominal.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_service_no_connection
    test/test_ff_service_no_connection.cc
  )
  target_link_libraries(test_ff_service_no_connection test_ff_service)
  add_ros_test(test/launch/test_ff_service_no_connection.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # ff_action
  ament_add_gtest_executable(test_ff_action_nominal_behavior
    test/test_ff_action_nominal_behavior.cc
  )
  ament_target_dependencies(test_ff_action_nominal_behavior rclcpp_action)
  target_link_libraries(test_ff_action_nominal_behavior ff_util)
  add_ros_test(test/launch/test_ff_action_nominal_behavior.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_active_timeout
    test/test_ff_action_active_timeout.cc
  )
  ament_target_dependencies(test_ff_action_active_timeout rclcpp_action)
  target_link_libraries(test_ff_action_active_timeout ff_util)
  add_ros_test(test/launch/test_ff_action_active_timeout.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_cancel_own_goal
    test/test_ff_action_cancel_own_goal.cc
  )
  ament_target_dependencies(test_ff_action_cancel_own_goal rclcpp_action)
  target_link_libraries(test_ff_action_cancel_own_goal ff_util)
  add_ros_test(test/launch/test_ff_action_cancel_own_goal.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_connect_timeout
    test/test_ff_action_connect_timeout.cc
  )
  ament_target_dependencies(test_ff_action_connect_timeout rclcpp_action)
  target_link_libraries(test_ff_action_connect_timeout ff_util)
  add_ros_test(test/launch/test_ff_action_connect_timeout.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_deadline_timeout
    test/test_ff_action_deadline_timeout.cc
  )
  ament_target_dependencies(test_ff_action_deadline_timeout rclcpp_action)
  target_link_libraries(test_ff_action_deadline_timeout ff_util)
  add_ros_test(test/launch/test_ff_action_deadline_timeout.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_preempt_others_goal
    test/test_ff_action_preempt_others_goal.cc
  )
  ament_target_dependencies(test_ff_action_preempt_others_goal rclcpp_action)
  target_link_libraries(test_ff_action_preempt_others_goal ff_util)
  add_ros_test(test/launch/test_ff_action_preempt_others_goal.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_preempt_own_goal
    test/test_ff_action_preempt_own_goal.cc
  )
  ament_target_dependencies(test_ff_action_preempt_own_goal rclcpp_action)
  target_link_libraries(test_ff_action_preempt_own_goal ff_util)
  add_ros_test(test/launch/test_ff_action_preempt_own_goal.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_ff_action_response_timeout
    test/test_ff_action_response_timeout.cc
  )
  ament_target_dependencies(test_ff_action_response_timeout rclcpp_action)
  target_link_libraries(test_ff_action_response_timeout ff_util)
  add_ros_test(test/launch/test_ff_action_response_timeout.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # latched topics
  ament_add_gtest_executable(test_latched_topics
    test/test_latched_topics.cc
  )
  target_link_libraries(test_latched_topics ff_util)
  add_ros_test(test/launch/test_latched_topics.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()


#############
## Install ##
#############

ament_export_dependencies(
  config_reader
  msg_conversions
  ff_common
)

ament_export_include_directories(include)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark libraries for installation
install(TARGETS ff_util
  EXPORT ff_util
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS test_ff_service
  EXPORT test_ff_service
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS test_ff_client
  EXPORT test_ff_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
