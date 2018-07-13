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

project(astrobee_gazebo)

# Find packages
find_package(gazebo REQUIRED)

# Add gazebo to the link directorries
link_directories(${GAZEBO_LIBRARY_DIRS})

# Call catkin
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gazebo_model_plugin_dock
    gazebo_model_plugin_truth
    gazebo_model_plugin_eps
    gazebo_model_plugin_pmc
    gazebo_model_plugin_perching_arm
    gazebo_model_plugin_laser
    gazebo_model_plugin_flashlights
    gazebo_sensor_plugin_imu
    gazebo_sensor_plugin_nav_cam
    gazebo_sensor_plugin_dock_cam
    gazebo_sensor_plugin_perch_cam
    gazebo_sensor_plugin_haz_cam
    gazebo_sensor_plugin_sparse_map
    gazebo_sensor_plugin_ar_tags
    gazebo_sensor_plugin_optical_flow
    gazebo_system_plugin_server
    gazebo_system_plugin_client
  CATKIN_DEPENDS
    visualization_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_msgs
    gazebo_ros
    sensor_msgs
    ff_msgs
    ff_hw_msgs
    ff_util
)

# Create a model plugin for the ground truth
create_library(
  DIR
    src
  TARGET
    astrobee_gazebo
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    ff_nodelet
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for the EPS system
create_library(
  DIR
    src/gazebo_model_plugin_eps
  TARGET
    gazebo_model_plugin_eps
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
    ff_hw_msgs
)

# Create a model plugin for aerodynamic drag
create_library(
  DIR
    src/gazebo_model_plugin_dock
  TARGET
    gazebo_model_plugin_dock
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for aerodynamic drag
create_library(
  DIR
    src/gazebo_model_plugin_drag
  TARGET
    gazebo_model_plugin_drag
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for the ground truth
create_library(
  DIR
    src/gazebo_model_plugin_truth
  TARGET
    gazebo_model_plugin_truth
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for the PMC
create_library(
  DIR
    src/gazebo_model_plugin_pmc
  TARGET
    gazebo_model_plugin_pmc
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
    gnc_autocode
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for the perching arm
create_library(
  DIR
    src/gazebo_model_plugin_perching_arm
  TARGET
    gazebo_model_plugin_perching_arm
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for the flashlights
create_library(
  DIR
    src/gazebo_model_plugin_flashlights
  TARGET
    gazebo_model_plugin_flashlights
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for the laser
create_library(
  DIR
    src/gazebo_model_plugin_laser
  TARGET
    gazebo_model_plugin_laser
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a sensor plugin for the IMU
create_library(
  DIR
    src/gazebo_sensor_plugin_imu
  TARGET
    gazebo_sensor_plugin_imu
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a sensor plugin for depth cameras
create_library(
  DIR
    src/gazebo_sensor_plugin_perch_cam
  TARGET
    gazebo_sensor_plugin_perch_cam
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a sensor plugin for depth cameras
create_library(
  DIR
    src/gazebo_sensor_plugin_haz_cam
  TARGET
    gazebo_sensor_plugin_haz_cam
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a sensor plugin for the nav cam
create_library(
  DIR
    src/gazebo_sensor_plugin_nav_cam
  TARGET
    gazebo_sensor_plugin_nav_cam
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
    camera
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
    ff_msgs
)

# Create a sensor plugin for the dock cam
create_library(
  DIR
    src/gazebo_sensor_plugin_dock_cam
  TARGET
    gazebo_sensor_plugin_dock_cam
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
)

# Create a model plugin for sparse mapping
create_library(
  DIR
    src/gazebo_sensor_plugin_sparse_map
  TARGET
    gazebo_sensor_plugin_sparse_map
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
    camera
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
    ff_msgs
)

# Create a model plugin for AR localization
create_library(
  DIR
    src/gazebo_sensor_plugin_ar_tags
  TARGET
    gazebo_sensor_plugin_ar_tags
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
    camera
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
    ff_msgs
)

# Create a model plugin for AR localization
create_library(
  DIR
    src/gazebo_sensor_plugin_optical_flow
  TARGET
    gazebo_sensor_plugin_optical_flow
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
    astrobee_gazebo
    camera
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  DEPS
    ff_util
    ff_msgs
)

# Create a model plugin for sparse mapping
create_library(
  DIR
    src/gazebo_system_plugin_server
  TARGET
    gazebo_system_plugin_server
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


# Create a model plugin for sparse mapping
create_library(
  DIR
    src/gazebo_system_plugin_client
  TARGET
    gazebo_system_plugin_client
  LIBS
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
  INC
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


# Install our launch files
install_launch_files()

# Also install some other resources
install(DIRECTORY media  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/spawn_model scripts/perf_wrapper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
