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
project(mobility)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -O3 -fPIC" )

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)
# find_package(mapper REQUIRED)
find_package(planner_trapezoidal REQUIRED)
# find_package(planner_qp REQUIRED)
find_package(jsonloader REQUIRED)


###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable: plangen
# add_executable(plangen tools/plangen.cc)
# add_dependencies(plangen ${catkin_EXPORTED_TARGETS})
# target_link_libraries(plangen
#   gflags glog ${catkin_LIBRARIES})

## Declare a C++ executable: teleop
add_executable(teleop tools/teleop.cc)
ament_target_dependencies(teleop rclcpp ff_common ff_util ff_msgs jsonloader)
target_link_libraries(teleop
  gflags glog)

#############
## Install ##
#############

# Install C++ executables
# install(TARGETS plangen DESTINATION bin)
install(TARGETS teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS ${PROJECT_NAME} teleop
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# install(CODE "execute_process(
#   COMMAND ln -s ../../bin/plangen share/${PROJECT_NAME}
#   COMMAND ln -s ../../bin/teleop share/${PROJECT_NAME}
#   WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
#   OUTPUT_QUIET
#   ERROR_QUIET
#   )")

ament_package()