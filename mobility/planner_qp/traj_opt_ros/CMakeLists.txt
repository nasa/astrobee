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
project(traj_opt_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations  -Werror=return-type -DQT_NO_KEYWORDS")

# Qt Stuff, assume Qt5
set(CMAKE_AUTOMOC ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../../cmake")
set(LIBRARIES traj_ros_bridge)

# build rviz plugin if not cross compiling
if(NOT USE_CTC)
  set(CMAKE_AUTOMOC ON)
  find_package(catkin2 COMPONENTS rviz QUIET)
  if(rviz_QT_VERSION)
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
    set(QT_LIBRARIES Qt5::Widgets)
    set(LIBRARIES ${LIBRARIES} traj_opt_ros)
    set(INCLUDES ${INCLUDES} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  endif()
endif()

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  traj_opt_basic
  traj_opt_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${LIBRARIES}
  CATKIN_DEPENDS
    roscpp
    traj_opt_basic
    traj_opt_msgs
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDES}
)

add_library(traj_ros_bridge
  src/ros_bridge.cpp)
target_link_libraries(traj_ros_bridge ${catkin_LIBRARIES})

if (NOT USE_CTC)
  if(rviz_QT_VERSION)
    add_library(${PROJECT_NAME}
      src/trajectory_visual.cpp
      src/trajectory_display.cpp
      ${MOC_FILES})
    target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} traj_ros_bridge)
  endif()
endif (NOT USE_CTC)


#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS traj_ros_bridge
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
