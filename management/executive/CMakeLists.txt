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
project(executive)

## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

## Find ament and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_hw_msgs REQUIRED)
find_package(config_reader REQUIRED)
find_package(ff_util REQUIRED)
find_package(jsonloader REQUIRED)

# System dependencies are found with CMake's conventions
#find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

# Find jsoncpp
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(JsonCpp REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${JSONCPP_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(sequencer SHARED
  src/utils/sequencer/command_conversion.cc
  src/utils/sequencer/plan_io.cc
  src/utils/sequencer/sequencer.cc
)
ament_target_dependencies(sequencer rclcpp rclcpp_components ff_msgs ff_hw_msgs config_reader ff_util jsonloader)

# Declare C++ libraries
add_library(executive SHARED
  src/executive.cc
  src/op_state.cc
  src/op_state_auto_return.cc
  src/op_state_fault.cc
  src/op_state_plan_exec.cc
  src/op_state_ready.cc
  src/op_state_repo.cc
  src/op_state_teleop.cc
)
target_compile_definitions(executive PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(executive rclcpp rclcpp_components ff_msgs ff_hw_msgs config_reader ff_util jsonloader)
rclcpp_components_register_nodes(executive "executive::Executive")

## Declare a C++ executable: data_to_disk_pub
#add_executable(data_to_disk_pub tools/data_to_disk_pub.cc)
#add_dependencies(data_to_disk_pub ${catkin_EXPORTED_TARGETS})
#target_link_libraries(data_to_disk_pub
#  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: ekf_switch_mux
#add_executable(ekf_switch_mux tools/ekf_switch_mux.cc)
#add_dependencies(ekf_switch_mux ${catkin_EXPORTED_TARGETS})
#target_link_libraries(ekf_switch_mux
#  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: plan_pub
#add_executable(plan_pub tools/plan_pub.cc)
#add_dependencies(plan_pub ${catkin_EXPORTED_TARGETS})
#target_link_libraries(plan_pub
#  executive gflags ${Boost_LIBRARIES})

## Declare a C++ executable: simple_move
#add_executable(simple_move tools/simple_move.cc)
#add_dependencies(simple_move ${catkin_EXPORTED_TARGETS})
#target_link_libraries(simple_move
#  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: teleop_tool
#add_executable(teleop_tool tools/teleop_tool.cc)
#add_dependencies(teleop_tool ${catkin_EXPORTED_TARGETS})
#target_link_libraries(teleop_tool
#  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: zones_pub
#add_executable(zones_pub tools/zones_pub.cc)
#add_dependencies(zones_pub ${catkin_EXPORTED_TARGETS})
#target_link_libraries(zones_pub
#  executive gflags ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  # Executive initialization fault tester
#  ament_add_gtest_executable(test_init_executive
#    test/test_init_executive.cc
#  )
#  target_link_libraries(test_init_executive execute)
#  add_ros_test(test/test_init_executive.py TIMEOUT "30" ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

#############
## Install ##
#############

ament_export_include_directories(include)

# Mark libraries for installation
install(TARGETS sequencer
  EXPORT sequencer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT executive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install C++ executables
#install(TARGETS data_to_disk_pub DESTINATION bin)
#install(TARGETS ekf_switch_mux DESTINATION bin)
#install(TARGETS plan_pub DESTINATION bin)
#install(TARGETS simple_move DESTINATION bin)
#install(TARGETS teleop_tool DESTINATION bin)
#install(TARGETS zones_pub DESTINATION bin)
#install(CODE "execute_process(
#  COMMAND ln -s ../../bin/data_to_disk_pub share/${PROJECT_NAME}
#  COMMAND ln -s ../../bin/ekf_switch_mux share/${PROJECT_NAME}
#  COMMAND ln -s ../../bin/plan_pub share/${PROJECT_NAME}
#  COMMAND ln -s ../../bin/simple_move share/${PROJECT_NAME}
#  COMMAND ln -s ../../bin/teleop_tool share/${PROJECT_NAME}
#  COMMAND ln -s ../../bin/zones_pub share/${PROJECT_NAME}
#  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
#  OUTPUT_QUIET
#  ERROR_QUIET
#  )")

# Mark launch files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()