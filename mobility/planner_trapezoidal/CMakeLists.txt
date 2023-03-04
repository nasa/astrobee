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
project(planner_trapezoidal)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

## Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)

# find_package(choreographer REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(mapper REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/planner_trapezoidal.cc
  src/planner_trapezoidal_component.cc
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components ff_msgs ff_common ff_util PCL)
# rclcpp_components_register_nodes(${PROJECT_NAME} "planner_trapezoidal::PlannerTrapezoidalComponent")

# add_dependencies(planner_trapezoidal ${catkin_EXPORTED_TARGETS})
# target_link_libraries(planner_trapezoidal ${catkin_LIBRARIES})

#############
## Install ##
#############
ament_export_dependencies(ament_cmake rclcpp)
ament_export_include_directories(include)

# Mark libraries for installation
install(TARGETS
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE)

ament_package()
