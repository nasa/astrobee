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

project(visualeyez)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    tf2
    tf2_ros
    geometry_msgs
    visualization_msgs
    std_srvs
)

create_library(TARGET visualeyez
  LIBS ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
)

create_tool_targets(
  DIR tools
  LIBS visualeyez
)

# Only test if it is enabled
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # We need to use the common client interface
  include_directories(${catkin_INCLUDE_DIRS} include)

  # A simple test to check that the streaming client receives, decodes 
  # and fails gracefully if a packet is malformed.
  add_rostest_gtest(test_visualeyez_stream
    test/test_visualeyez_stream.test
    test/test_visualeyez_stream.cc)
  target_link_libraries(test_visualeyez_stream visualeyez)

  # A simple test to check that the streaming client receives, decodes 
  # and fails gracefully if a packet is malformed.
  add_rostest_gtest(test_visualeyez_util
    test/test_visualeyez_util.test
    test/test_visualeyez_util.cc)
  target_link_libraries(test_visualeyez_util visualeyez)

endif()

install_launch_files()