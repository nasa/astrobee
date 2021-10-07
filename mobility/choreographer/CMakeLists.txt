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
project(choreographer)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  ff_util
  mapper
  ff_msgs
  ff_hw_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    choreographer
  CATKIN_DEPENDS
    roscpp
    nodelet
    mapper
    ff_msgs
    ff_util
    tf2
    tf2_ros
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
add_library(choreographer
  src/choreographer_nodelet.cc
  src/validator.cc
)
add_dependencies(choreographer ${catkin_EXPORTED_TARGETS})
target_link_libraries(choreographer ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Choreographer initialization fault tester
  add_rostest_gtest(test_init_choreographer
    test/test_init_choreographer.test
    test/test_init_choreographer.cc
  )
  target_link_libraries(test_init_choreographer
    ${catkin_LIBRARIES} glog
  )
  if(ENABLE_INTEGRATION_TESTING)
    # Choreographer test obstacles
    add_rostest_gtest(test_obstacle
      test/test_obstacle.test
      test/test_obstacle.cc
    )

    target_link_libraries(test_obstacle
      ${catkin_LIBRARIES}
    )

    # Choreographer test zones keepin
    add_rostest_gtest(test_zones_keepin
      test/test_zones_keepin.test
      test/test_zones_keepin.cc
    )

    target_link_libraries(test_zones_keepin
      ${catkin_LIBRARIES}
    )

    # Choreographer test zones keepout
    add_rostest_gtest(test_zones_keepout
      test/test_zones_keepout.test
      test/test_zones_keepout.cc
    )

    target_link_libraries(test_zones_keepout
      ${catkin_LIBRARIES}
    )

    # Choreographer test zones nominal
    add_rostest_gtest(test_zones_nominal
      test/test_zones_nominal.test
      test/test_zones_nominal.cc
    )

    target_link_libraries(test_zones_nominal
      ${catkin_LIBRARIES}
    )
  endif()
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

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
