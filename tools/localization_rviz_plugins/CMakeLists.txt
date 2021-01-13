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

cmake_minimum_required(VERSION 2.8.3)

project(localization_rviz_plugins)

set(BUILD_LOC_RVIZ_PLUGINS ON CACHE BOOL "Ignore loc rviz plugins due to cross compile error" FORCE)

if(BUILD_LOC_RVIZ_PLUGINS)
 
 # For Qt
 set(CMAKE_AUTOMOC ON)
 
 find_package(catkin REQUIRED COMPONENTS rviz)
 
 find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
 set(QT_LIBRARIES Qt5::Widgets)

 
 catkin_package(
   LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} ${QT_LIBRARIES} camera config_reader graph_bag graph_localizer imu_integration localization_common localization_measurements 
   INCLUDE_DIRS ${GTSAM_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} 
   CATKIN_DEPENDS rviz 
   DEPENDS gtsam 
 )
 
 create_library(TARGET ${PROJECT_NAME} 
   LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} ${QT_LIBRARIES} camera config_reader graph_bag graph_localizer imu_integration localization_common localization_measurements 
   INC ${catkin_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} 
   ADD_SRCS src/localization_graph_display.cc src/localization_graph_panel.cc src/pose_display.cc src/utilities.cc src/imu_augmentor_display.cc src/sparse_mapping_display.cc ${MOC_FILES}
   DEPS gtsam rviz 
 )

endif()
