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

project(choreographer)

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
    nav_msgs
    geometry_msgs
    visualization_msgs
)

create_library(TARGET choreographer
  LIBS ${catkin_LIBRARIES} ff_common ff_nodelet ff_flight config_server mapper
  INC  ${catkin_INCLUDE_DIRS}
  DEPS ff_common nav_msgs geometry_msgs ff_msgs ff_util mapper
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Choreographer initialization fault tester
  add_rostest_gtest(test_init_choreographer
    test/test_init_choreographer.test
    test/test_init_choreographer.cc
  )
  target_link_libraries(test_init_choreographer
    ${catkin_LIBRARIES} config_reader ff_nodelet
  )
  if(ENABLE_INTEGRATION_TESTING)
    # Choreographer test obstacles
    add_rostest_gtest(test_obstacle
      test/test_obstacle.test
      test/test_obstacle.cc
    )

    target_link_libraries(test_obstacle
      ${catkin_LIBRARIES} config_client ff_nodelet
    )

    # Choreographer test zones keepin
    add_rostest_gtest(test_zones_keepin
      test/test_zones_keepin.test
      test/test_zones_keepin.cc
    )

    target_link_libraries(test_zones_keepin
      ${catkin_LIBRARIES} config_client ff_nodelet
    )

    # Choreographer test zones keepout
    add_rostest_gtest(test_zones_keepout
      test/test_zones_keepout.test
      test/test_zones_keepout.cc
    )

    target_link_libraries(test_zones_keepout
      ${catkin_LIBRARIES} config_client ff_nodelet
    )

    # Choreographer test zones nominal
    add_rostest_gtest(test_zones_nominal
      test/test_zones_nominal.test
      test/test_zones_nominal.cc
    )

    target_link_libraries(test_zones_nominal
      ${catkin_LIBRARIES} config_client ff_nodelet
    )
  endif()
endif()

install_launch_files()
