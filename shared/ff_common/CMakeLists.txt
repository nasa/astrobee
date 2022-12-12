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

project(ff_common)

cmake_minimum_required(VERSION 3.5)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)

# Build for ROS1
if("$ENV{ROS_VERSION}" STREQUAL "1")

  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    roscpp
    std_msgs
    nodelet
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ff_common
    CATKIN_DEPENDS roscpp std_msgs nodelet
  )

# Build for ROS2
elseif("$ENV{ROS_VERSION}" STREQUAL "2")

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  set(CATKIN_PACKAGE_SHARE_DESTINATION "share/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_BIN_DESTINATION "lib/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_LIB_DESTINATION "lib/${PROJECT_NAME}")
  set(CATKIN_PACKAGE_INCLUDE_DESTINATION "include/${PROJECT_NAME}")

endif()

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} # ROS1
  ${rclcpp_INCLUDE_DIRS} # ROS2
  ${EIGEN3_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(ff_common
  src/init.cc
  src/thread.cc
  src/utils.cc
)
target_link_libraries(ff_common gflags glog ${catkin_LIBRARIES} )

if("$ENV{ROS_VERSION}" STREQUAL "1")
  add_dependencies(ff_common ${catkin_EXPORTED_TARGETS})

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_thread
      test/test_thread.test
      test/test_thread.cc
    )
    target_link_libraries(test_thread
      ff_common
    )
  endif()

elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  ament_target_dependencies(ff_common rclcpp)
  ament_export_targets(ff_common_export HAS_LIBRARY_TARGET)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_thread
      test/test_thread.test
      test/test_thread.cc
    )
    target_link_libraries(test_thread
      ff_common
    )
  endif()

  ament_export_include_directories(include)
  ament_package()

endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT ff_common_export
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)