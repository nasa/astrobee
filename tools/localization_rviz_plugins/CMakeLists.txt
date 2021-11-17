#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.0)
project(localization_rviz_plugins)

if (USE_ROS AND BUILD_LOC_RVIZ_PLUGINS)
 
  ## Compile as C++14, supported in ROS Kinetic and newer
  add_compile_options(-std=c++14)


  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    localization_common
    ff_util
    graph_localizer
    graph_bag
    rviz
  )

  # For Qt
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)

 catkin_package(
   LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} 
   INCLUDE_DIRS ${GTSAM_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS}
   CATKIN_DEPENDS roscpp localization_common ff_util graph_localizer graph_bag rviz
 )
 
  ###########
  ## Build ##
  ###########
  # Specify additional locations of header files
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
  )

  # Declare C++ libraries
  add_library(${PROJECT_NAME}
    src/depth_odometry_display.cc
    src/localization_graph_display.cc
    src/localization_graph_panel.cc
    src/pose_display.cc
    src/utilities.cc
    src/imu_augmentor_display.cc
    src/sparse_mapping_display.cc
  )
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

else (USE_ROS AND BUILD_LOC_RVIZ_PLUGINS)
  find_package(catkin REQUIRED COMPONENTS)
  catkin_package()
endif (USE_ROS AND BUILD_LOC_RVIZ_PLUGINS)
