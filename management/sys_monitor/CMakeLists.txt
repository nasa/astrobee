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

cmake_minimum_required(VERSION 3.0)
project(sys_monitor)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  ff_util
  ff_msgs
  nodelet
  config_reader
)

catkin_package(
  LIBRARIES sys_monitor
  DEPENDS roscpp ff_util ff_msgs nodelet config_reader
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(sys_monitor
  src/sys_monitor.cc
)
add_dependencies(sys_monitor ${catkin_EXPORTED_TARGETS})
target_link_libraries(sys_monitor ${catkin_LIBRARIES})

## Declare a C++ executable: fault_tester
add_executable(fault_tester tools/fault_tester.cc)
add_dependencies(fault_tester ${catkin_EXPORTED_TARGETS})
target_link_libraries(fault_tester
  sys_monitor gflags glog ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # System monitor initialization fault tester
  add_rostest_gtest(test_init_sys_monitor
    test/test_init_sys_monitor.test
    test/test_init_sys_monitor.cc
  )

  target_link_libraries(test_init_sys_monitor
    ${catkin_LIBRARIES} glog
  )
  # System monitor tester
  add_rostest_gtest(test_sys_monitor
    test/test_sys_monitor.test
    test/test_sys_monitor.cc
  )

  target_link_libraries(test_sys_monitor
    ${catkin_LIBRARIES} glog
  )
endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install C++ executables
install(TARGETS fault_tester DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/fault_tester share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)