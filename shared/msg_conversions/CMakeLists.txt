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
project(msg_conversions)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Build for ROS1
if("$ENV{ROS_VERSION}" STREQUAL "1")
  add_compile_definitions(ROS1)
  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
    config_reader
    ff_msgs
    tf2
    tf2_ros
  )

  find_package(Eigen3 REQUIRED)

  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs config_reader tf2 tf2_ros
  )

# Build for ROS2
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  add_compile_definitions(ROS2)
  # Find amend and libraries
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)

  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(config_reader REQUIRED)
  find_package(ff_common REQUIRED)
  find_package(ff_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)


  set(CATKIN_PACKAGE_SHARE_DESTINATION "share/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_BIN_DESTINATION "lib/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_LIB_DESTINATION "lib/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_INCLUDE_DESTINATION "include")

endif()

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}         # ROS1
  ${config_reader_INCLUDE_DIRS}  # ROS2
)

# Declare C++ libraries
add_library(msg_conversions
  src/msg_conversions.cc
)
target_link_libraries(msg_conversions ${EIGEN_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ executable: landmark_msg_cnv
add_executable(landmark_msg_cnv tools/landmark_msg_cnv.cc)
target_link_libraries(landmark_msg_cnv
  msg_conversions gflags ${catkin_LIBRARIES})

## Declare a C++ executable: pose_stamped_msg_cnv
add_executable(pose_stamped_msg_cnv tools/pose_stamped_msg_cnv.cc)
target_link_libraries(pose_stamped_msg_cnv
  msg_conversions gflags ${catkin_LIBRARIES})


if("$ENV{ROS_VERSION}" STREQUAL "1")
add_dependencies(msg_conversions ${catkin_EXPORTED_TARGETS})

add_dependencies(landmark_msg_cnv ${catkin_EXPORTED_TARGETS})

add_dependencies(pose_stamped_msg_cnv ${catkin_EXPORTED_TARGETS})

elseif("$ENV{ROS_VERSION}" STREQUAL "2")
ament_target_dependencies(msg_conversions rclcpp geometry_msgs ff_msgs)

ament_target_dependencies(landmark_msg_cnv rclcpp ff_common)

ament_target_dependencies(pose_stamped_msg_cnv rclcpp ff_common tf2_ros)

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

# Install C++ executables
install(TARGETS landmark_msg_cnv DESTINATION bin)
install(TARGETS pose_stamped_msg_cnv DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/landmark_msg_cnv share/${PROJECT_NAME}
  COMMAND ln -s ../../bin/pose_stamped_msg_cnv share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
