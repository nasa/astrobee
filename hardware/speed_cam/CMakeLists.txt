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
project(speed_cam)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_util
  ff_hw_msgs
  ff_serial
  mavlink
)

catkin_package(
  LIBRARIES
    speed_cam
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_util
    ff_hw_msgs
    ff_serial
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
# Declare C++ libraries
add_library(speed_cam
  src/speed_cam.cc
  src/speed_cam_node.cc
)
add_dependencies(speed_cam ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_cam ${catkin_LIBRARIES})

## Declare a C++ executable: speed_cam_tool
add_executable(speed_cam_tool tools/speed_cam_tool.cc)
add_dependencies(speed_cam_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_cam_tool
  speed_cam gflags ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Speed cam initialization fault tester
  add_rostest_gtest(test_init_speed_cam
    test/test_init_speed_cam.test
    test/test_init_speed_cam.cc
  )

  target_link_libraries(test_init_speed_cam
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
install(TARGETS speed_cam_tool DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/speed_cam_tool share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

