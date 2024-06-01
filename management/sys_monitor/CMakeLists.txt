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
project(sys_monitor)

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
find_package(config_reader REQUIRED)
find_package(ff_util REQUIRED)


###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

# Declare C++ libraries
add_library(sys_monitor SHARED
  src/sys_monitor.cc
)
ament_target_dependencies(sys_monitor rclcpp rclcpp_components ff_msgs config_reader ff_util)

## Declare a C++ executable: fault_tester
#add_executable(fault_tester tools/fault_tester.cc)
#ament_target_dependencies(fault_tester rclcpp rclcpp_components ff_msgs config_reader ff_util)
#target_link_libraries(fault_tester sys_monitor gflags glog)


#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  # System monitor initialization fault tester
#  add_rostest_gtest(test_init_sys_monitor
#    test/test_init_sys_monitor.test
#    test/test_init_sys_monitor.cc
#  )

#  target_link_libraries(test_init_sys_monitor
#    ${catkin_LIBRARIES} glog
#  )
#  # System monitor tester
#  add_rostest_gtest(test_sys_monitor
#    test/test_sys_monitor.test
#    test/test_sys_monitor.cc
#  )

#  target_link_libraries(test_sys_monitor
#    ${catkin_LIBRARIES} glog
#  )
#endif()

#############
## Install ##
#############

ament_export_include_directories(include)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT sys_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# Install C++ executables
#install(TARGETS fault_tester RUNTIME DESTINATION lib/${PROJECT_NAME})
#install(CODE "execute_process(
#  COMMAND ln -s ../../bin/fault_tester share/${PROJECT_NAME}
#  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
#  OUTPUT_QUIET
#  ERROR_QUIET
#  )")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE  
)

# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)

ament_package()
