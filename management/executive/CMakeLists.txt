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

cmake_minimum_required(VERSION 2.8.12) # Need get_filename_component DIRECTORY
project(executive)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ff_msgs
  ff_hw_msgs
  nodelet
  config_reader
  ff_util
)

# System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

# Find jsoncpp
find_package(jsoncpp REQUIRED)
find_path(
  JSONCPP_INCLUDE_DIR 
  NAMES jsoncpp/json/json.h json/json.h
  DOC "jsoncpp include dir"
)
if (EXISTS "${JSONCPP_INCLUDE_DIR}/jsoncpp/json/json.h" )
  set (JSONCPP_INCLUDE_DIR "${JSONCPP_INCLUDE_DIR}/jsoncpp")
endif ()


catkin_package(
  LIBRARIES
    executive
    sequencer
  CATKIN_DEPENDS
    roscpp
    ff_msgs
    ff_hw_msgs
    ff_util
    nodelet
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(sequencer
  src/utils/sequencer/command_conversion.cc
  src/utils/sequencer/plan_io.cc
  src/utils/sequencer/sequencer.cc
)
add_dependencies(sequencer ${catkin_EXPORTED_TARGETS})
target_link_libraries(sequencer ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(executive
  src/executive.cc
  src/op_state.cc
  src/op_state_auto_return.cc
  src/op_state_fault.cc
  src/op_state_plan_exec.cc
  src/op_state_ready.cc
  src/op_state_repo.cc
  src/op_state_teleop.cc
)
add_dependencies(executive ${catkin_EXPORTED_TARGETS})
target_link_libraries(executive sequencer ${catkin_LIBRARIES})


## Declare a C++ executable: ekf_switch_mux
add_executable(ekf_switch_mux tools/ekf_switch_mux.cc)
add_dependencies(ekf_switch_mux ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_switch_mux
  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: plan_pub
add_executable(plan_pub tools/plan_pub.cc)
add_dependencies(plan_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_pub
  executive gflags ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a C++ executable: simple_move
add_executable(simple_move tools/simple_move.cc)
add_dependencies(simple_move ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move
  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: teleop_tool
add_executable(teleop_tool tools/teleop_tool.cc)
add_dependencies(teleop_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_tool
  executive gflags ${catkin_LIBRARIES})

## Declare a C++ executable: zones_pub
add_executable(zones_pub tools/zones_pub.cc)
add_dependencies(zones_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(zones_pub
  executive gflags ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Executive initialization fault tester
  add_rostest_gtest(test_init_executive
    test/test_init_executive.test
    test/test_init_executive.cc
  )

  target_link_libraries(test_init_executive
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
endif()

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/ DESTINATION config)
