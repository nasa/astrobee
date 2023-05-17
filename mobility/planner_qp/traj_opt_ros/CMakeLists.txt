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
project(traj_opt_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations  -Werror=return-type -DQT_NO_KEYWORDS")

# ## Qt Stuff, assume Qt5
# set(CMAKE_AUTOMOC ON)

# ## Find catkin macros and libraries
# set(LIBRARIES traj_ros_bridge)

## Find amend macros and libraries
set(dependencies 
rclcpp
rclcpp_components
ff_common
ff_msgs
ff_util
traj_opt_basic
traj_opt_msgs
)

find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Core)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

# build rviz plugin if not cross compiling
# if(NOT USE_CTC="${USE_CTC}")
#   # message("Using CTC")
#   set(CMAKE_AUTOMOC ON)
#   # find_package(catkin2 COMPONENTS rviz QUIET)
#   find_package(rviz2 QUIET)
#   if(rviz2_QT_VERSION)
#     message(STATUS "Using Qt5 based on the rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
#     find_package(Qt5 ${rviz2_QT_VERSION} REQUIRED Core Widgets)
#     set(QT_LIBRARIES Qt5::Widgets)
#     set(LIBRARIES ${LIBRARIES} traj_opt_ros)
#     set(INCLUDES ${INCLUDES} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})
#   endif()
# endif()

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${rviz2_INCLUDE_DIRS}
  ${INCLUDES}
)

add_library(traj_ros_bridge
  src/ros_bridge.cpp
)

ament_target_dependencies(traj_ros_bridge ${QT_LIBRARIES} ${dependencies})

target_compile_definitions(traj_ros_bridge PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(traj_ros_bridge ${dependencies})
ament_export_targets(traj_ros_bridge HAS_LIBRARY_TARGET)

# if (NOT USE_CTC)
#   message("Using CTC BUILD")
#   if(rviz2_QT_VERSION)
#     add_library(${PROJECT_NAME}
#       src/trajectory_visual.cpp
#       src/trajectory_display.cpp
#       ${MOC_FILES})
#     target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${rviz2_DEFAULT_PLUGIN_LIBRARIES} traj_ros_bridge)
#   endif()
# endif (NOT USE_CTC)

#############
## Install ##
#############

# target_include_directories(traj_ros_bridge
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# Mark libraries for installation
install(TARGETS traj_ros_bridge
  EXPORT traj_ros_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# if (NOT USE_CTC)
#   if(rviz2_QT_VERSION)
#     install(TARGETS ${PROJECT_NAME}
#       ARCHIVE DESTINATION lib
#       LIBRARY DESTINATION lib
#       RUNTIME DESTINATION bin
#     )
#   endif()
# endif (NOT USE_CTC)

# Mark cpp header files for installation
install(DIRECTORY launch/
  DESTINATION share/traj_ros_bridge
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()