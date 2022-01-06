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

cmake_minimum_required(VERSION 3.0.2)
project(interactive_marker_teleop)

if (BUILD_LOC_RVIZ_PLUGINS)

  ## Compile as C++14, supported in ROS Kinetic and newer
  add_compile_options(-std=c++14)

  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    ff_hw_msgs
    ff_msgs
    ff_util
    roscpp
    interactive_markers
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if your package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
    INCLUDE_DIRS include
  )

  ###########
  ## Build ##
  ###########

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
   
  ## Declare a C++ executable
  add_executable(${PROJECT_NAME} src/interactive_marker_teleop.cpp)
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  #############
  ## Install ##
  #############

  ## Mark executables for installation
  ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

else (BUILD_LOC_RVIZ_PLUGINS)
  find_package(catkin REQUIRED COMPONENTS)
  catkin_package()
endif (BUILD_LOC_RVIZ_PLUGINS)
