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
project(choreographer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_hw_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(jsonloader REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

# Declare C++ libraries
add_library(choreographer SHARED
  src/choreographer_component.cc
  src/validator.cc
)
target_compile_definitions(choreographer
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(choreographer rclcpp rclcpp_components ff_common ff_util ff_msgs ff_hw_msgs jsonloader geometry_msgs nav_msgs visualization_msgs)
rclcpp_components_register_nodes(choreographer "choreographer::ChoreographerComponent")

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  # Choreographer initialization fault tester
#  add_rostest_gtest(test_init_choreographer
#    test/test_init_choreographer.test
#    test/test_init_choreographer.cc
#  )
#  target_link_libraries(test_init_choreographer
#    ${catkin_LIBRARIES} glog
#  )
#  if(ENABLE_INTEGRATION_TESTING)
#    # Choreographer test obstacles
#    add_rostest_gtest(test_obstacle
#      test/test_obstacle.test
#      test/test_obstacle.cc
#    )
#
#    target_link_libraries(test_obstacle
#      ${catkin_LIBRARIES}
#    )
#
#    # Choreographer test zones keepin
#    add_rostest_gtest(test_zones_keepin
#      test/test_zones_keepin.test
#      test/test_zones_keepin.cc
#    )
#
#    target_link_libraries(test_zones_keepin
#      ${catkin_LIBRARIES}
#    )
#
#    # Choreographer test zones keepout
#    add_rostest_gtest(test_zones_keepout
#      test/test_zones_keepout.test
#      test/test_zones_keepout.cc
#    )
#
#    target_link_libraries(test_zones_keepout
#      ${catkin_LIBRARIES}
#    )
#
#    # Choreographer test zones nominal
#    add_rostest_gtest(test_zones_nominal
#      test/test_zones_nominal.test
#      test/test_zones_nominal.cc
#    )
#
#    target_link_libraries(test_zones_nominal
#      ${catkin_LIBRARIES}
#    )
#  endif()
#endif()

#############
## Install ##
#############

ament_export_include_directories(include)

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

ament_package()
