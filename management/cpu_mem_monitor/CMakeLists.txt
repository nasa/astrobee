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
project(cpu_mem_monitor)

## Compile as C++14, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_hw_msgs REQUIRED)
find_package(ff_util REQUIRED)
find_package(config_reader REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

# Declare C++ libraries
add_library(cpu_mem_monitor SHARED
  src/cpu.cc
  src/cpu_mem_monitor.cc
)

target_compile_definitions(cpu_mem_monitor
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(cpu_mem_monitor rclcpp rclcpp_components ff_common config_reader ff_msgs ff_util ff_hw_msgs)
rclcpp_components_register_nodes(cpu_mem_monitor "cpu_mem_monitor::CpuMemMonitor")

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  # llp CPU monitor initialization fault tester
#  add_rostest_gtest(test_init_llp_cpu_mem_monitor
#    test/test_init_llp_cpu_mem_monitor.test
#    test/test_init_llp_cpu_mem_monitor.cc
#  )
#
#  target_link_libraries(test_init_llp_cpu_mem_monitor
#    ${catkin_LIBRARIES} glog
#  )
#  # mlp CPU monitor initialization fault tester
#  add_rostest_gtest(test_init_mlp_cpu_mem_monitor
#    test/test_init_mlp_cpu_mem_monitor.test
#    test/test_init_mlp_cpu_mem_monitor.cc
#  )
#
#  target_link_libraries(test_init_mlp_cpu_mem_monitor
#    ${catkin_LIBRARIES} glog
#  )
#
#endif()

#############
## Install ##
#############

ament_export_include_directories(include)

target_include_directories(cpu_mem_monitor
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
install(TARGETS ${PROJECT_NAME}
  EXPORT cpu_mem_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
