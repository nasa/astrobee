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

# Check to see if correct version of media has been downloaded into tree. We
# dont do this if we are cross-compiling, as there is no need for it. We must
# also make sure the media is copied in a native install to a simulator.

cmake_minimum_required(VERSION 3.5)
project(description)


# Build for ROS1
if("$ENV{ROS_VERSION}" STREQUAL "1")

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

# Build for ROS2
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  # Find amend and libraries
  find_package(ament_cmake REQUIRED)

  set(CATKIN_PACKAGE_SHARE_DESTINATION "share/${PROJECT_NAME}")

  ament_package()
endif()


#############
## Install ##
#############

install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})