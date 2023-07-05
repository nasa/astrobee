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

set(dependencies
  rclcpp
  rclcpp_components
  ff_common
  ff_util
  ff_msgs
  choreographer
  tf2_geometry_msgs
  #mapper
)

## Find ament and libraries
find_package(ament_cmake REQUIRED)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()



## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common)

###########
## Build ##
###########

message(STATUS "Boost_INCLUDE_DIRS contains: ${Boost_INCLUDE_DIRS}")
# Specify additional locations of header files
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME} SHARED
  src/planner_trapezoidal.cc
  src/planner_trapezoidal_component.cc
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components ff_common ff_util ff_msgs PCL choreographer tf2_geometry_msgs)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
rclcpp_components_register_nodes(${PROJECT_NAME} "planner_trapezoidal::PlannerTrapezoidalComponent")

#############
## Install ##
#############

target_include_directories(planner_trapezoidal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)



# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT planner_trapezoidal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN ".svn" EXCLUDE)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
