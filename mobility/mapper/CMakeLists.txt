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
project(mapper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

## This fixes the Eigen aligment issue
## http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D EIGEN_MAX_STATIC_ALIGN_BYTES=0")

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_srvs REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)

  # message_runtime


LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(Octomap REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
# Find PCL
find_package(PCL REQUIRED COMPONENTS common)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME} SHARED
  src/callbacks.cc
  src/mapper_component.cc
  src/octoclass.cc
  src/polynomials.cc
  src/sampled_trajectory.cc
  src/services.cc
  src/threads.cc
  src/visualization_functions.cc
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components std_srvs ff_msgs ff_util visualization_msgs octomap)
rclcpp_components_register_nodes(${PROJECT_NAME} "mapper::MapperComponent")


if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  # Mapper initialization fault tester
  # add_rostest_gtest(test_init_mapper
  #   test/test_init_mapper.test
  #   test/test_init_mapper.cc
  # )

  # target_link_libraries(test_init_mapper
  #   ${catkin_LIBRARIES} glog
  # )
endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

ament_package()