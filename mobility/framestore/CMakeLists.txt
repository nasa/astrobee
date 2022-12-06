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
project(framestore)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Build for ROS1
if("$ENV{ROS_VERSION}" STREQUAL "1")
  add_compile_definitions(ROS1)
  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    roscpp
    nodelet
    pluginlib
    ff_util
    tf2_ros
    geometry_msgs
    config_reader
  )

  # Call catkin
  catkin_package(
    CATKIN_DEPENDS
      roscpp nodelet pluginlib ff_util tf2_ros geometry_msgs config_reader
  )

# Build for ROS2
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  add_compile_definitions(ROS2)

  # Find amend and libraries
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(rcutils REQUIRED)

  find_package(ff_common REQUIRED)
  find_package(ff_util REQUIRED)
  find_package(ff_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(config_reader REQUIRED)
  find_package(msg_conversions REQUIRED)

  set(CATKIN_PACKAGE_SHARE_DESTINATION "share/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_BIN_DESTINATION "lib/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_INCLUDE_DESTINATION "include/${PROJECT_NAME}")

endif()

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}         # ROS1
  ${ff_common_INCLUDE_DIRS}      # ROS2
  ${config_reader_INCLUDE_DIRS}  # ROS2
  ${msg_conversions_INCLUDE_DIRS}  # ROS2
)

# Declare C++ libraries
add_library(framestore
  src/framestore.cc
)
target_link_libraries(framestore ${catkin_LIBRARIES})


## Declare a C++ executable: global_transforms
add_executable(global_transforms tools/global_transforms.cc)
target_link_libraries(global_transforms
  framestore gflags glog ${catkin_LIBRARIES})

if("$ENV{ROS_VERSION}" STREQUAL "1")
add_dependencies(framestore ${catkin_EXPORTED_TARGETS})

add_dependencies(global_transforms ${catkin_EXPORTED_TARGETS})

elseif("$ENV{ROS_VERSION}" STREQUAL "2")
ament_target_dependencies(framestore rclcpp rclcpp_components ff_util config_reader msg_conversions)

ament_target_dependencies(global_transforms rclcpp config_reader msg_conversions tf2_ros)

ament_export_include_directories(include)
ament_package()

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
install(TARGETS global_transforms DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/global_transforms share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
