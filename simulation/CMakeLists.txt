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
project(astrobee_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC" )

# Build for ROS2
add_definitions(-DROS2)

# Find amend and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ff_common REQUIRED)
find_package(ff_util REQUIRED)
find_package(ff_msgs REQUIRED)
find_package(ff_hw_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


#   visualization_msgs
#   tf2
#   tf2_ros
#   tf2_eigen
#   tf2_msgs
#   gazebo_ros
#   sensor_msgs
#   ff_msgs
#   ff_hw_msgs
#   tf2_geometry_msgs
#   ff_util
#   gnc_autocode
#   camera
#   cv_bridge
#   jsonloader
# )

# Find packages
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# Find jsoncpp
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../cmake")
find_package(JsonCpp REQUIRED)


###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${config_reader_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
# Add gazebo to the link directorries
# link_directories(
#   ${gazebo_dev_LIBRARY_DIRS}
# )

# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create reusable classes for models and plugins
add_library(astrobee_gazebo SHARED
  src/astrobee_gazebo.cc
)
ament_target_dependencies(astrobee_gazebo rclcpp ff_common ff_util gazebo_dev gazebo_ros)

# Create a model plugin for the EPS system
add_library(gazebo_model_plugin_eps SHARED
  src/gazebo_model_plugin_eps/gazebo_model_plugin_eps.cc
)
target_link_libraries(gazebo_model_plugin_eps astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_eps rclcpp ff_hw_msgs tf2 tf2_ros gazebo_dev gazebo_ros)
ament_export_libraries(gazebo_model_plugin_eps)

# Create a model plugin for aerodynamic drag
add_library(gazebo_model_plugin_drag SHARED
  src/gazebo_model_plugin_drag/gazebo_model_plugin_drag.cc
)
target_link_libraries(gazebo_model_plugin_drag astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_drag rclcpp gazebo_dev gazebo_ros)
ament_export_libraries(gazebo_model_plugin_drag)

# Create a model plugin for the ground truth
add_library(gazebo_model_plugin_truth SHARED
  src/gazebo_model_plugin_truth/gazebo_model_plugin_truth.cc
)
target_link_libraries(gazebo_model_plugin_truth astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_truth rclcpp tf2 tf2_ros gazebo_dev gazebo_ros)
ament_export_libraries(gazebo_model_plugin_truth)

# Create a model plugin for the propulsion system
# add_library(gazebo_model_plugin_pmc SHARED
#   src/gazebo_model_plugin_pmc/gazebo_model_plugin_pmc.cc
# )
# target_link_libraries(gazebo_model_plugin_pmc astrobee_gazebo)
# ament_target_dependencies(gazebo_model_plugin_pmc rclcpp ff_msgs ff_hw_msgs gazebo_dev gazebo_ros)
# ament_export_libraries(gazebo_model_plugin_pmc)

# Create a model plugin for the perching arm
add_library(gazebo_model_plugin_perching_arm SHARED
  src/gazebo_model_plugin_perching_arm/gazebo_model_plugin_perching_arm.cc
)
target_link_libraries(gazebo_model_plugin_perching_arm astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_perching_arm rclcpp ff_msgs ff_hw_msgs gazebo_dev gazebo_ros)
ament_export_libraries(gazebo_model_plugin_perching_arm)

# Create a model plugin for the front flashlight
add_library(gazebo_model_plugin_flashlight SHARED
  src/gazebo_model_plugin_flashlight/gazebo_model_plugin_flashlight.cc
)
target_link_libraries(gazebo_model_plugin_flashlight astrobee_gazebo)
ament_target_dependencies(gazebo_model_plugin_flashlight rclcpp ff_hw_msgs visualization_msgs gazebo_dev gazebo_ros)
ament_export_libraries(gazebo_model_plugin_flashlight)

# Create a model plugin for the laser
# add_library(gazebo_model_plugin_laser
#   src/gazebo_model_plugin_laser/gazebo_model_plugin_laser.cc
# )
# add_dependencies(gazebo_model_plugin_laser ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_laser astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a model plugin for the LLP CPU mem monitor
# add_library(gazebo_model_plugin_llp_cpu_mem_monitor
#   src/gazebo_model_plugin_llp_cpu_mem_monitor/gazebo_model_plugin_llp_cpu_mem_monitor.cc
# )
# add_dependencies(gazebo_model_plugin_llp_cpu_mem_monitor ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_llp_cpu_mem_monitor astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a model plugin for the LLP disk monitor
# add_library(gazebo_model_plugin_llp_disk_monitor
#   src/gazebo_model_plugin_llp_disk_monitor/gazebo_model_plugin_llp_disk_monitor.cc
# )
# add_dependencies(gazebo_model_plugin_llp_disk_monitor ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_llp_disk_monitor astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a model plugin for the MLP CPU mem monitor
# add_library(gazebo_model_plugin_mlp_cpu_mem_monitor
#   src/gazebo_model_plugin_mlp_cpu_mem_monitor/gazebo_model_plugin_mlp_cpu_mem_monitor.cc
# )
# add_dependencies(gazebo_model_plugin_mlp_cpu_mem_monitor ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_mlp_cpu_mem_monitor astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a model plugin for the MLP disk monitor
# add_library(gazebo_model_plugin_mlp_disk_monitor
#   src/gazebo_model_plugin_mlp_disk_monitor/gazebo_model_plugin_mlp_disk_monitor.cc
# )
# add_dependencies(gazebo_model_plugin_mlp_disk_monitor ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_mlp_disk_monitor astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a model plugin for the speed camera
# add_library(gazebo_model_plugin_speed_cam
#   src/gazebo_model_plugin_speed_cam/gazebo_model_plugin_speed_cam.cc
# )
# add_dependencies(gazebo_model_plugin_speed_cam ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_speed_cam astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a model plugin for the signal lights
# add_library(gazebo_model_plugin_signal_lights
#   src/gazebo_model_plugin_signal_lights/gazebo_model_plugin_signal_lights.cc
# )
# add_dependencies(gazebo_model_plugin_signal_lights ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_model_plugin_signal_lights astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a sensor plugin for the IMU
# add_library(gazebo_sensor_plugin_imu
#   src/gazebo_sensor_plugin_imu/gazebo_sensor_plugin_imu.cc
# )
# add_dependencies(gazebo_sensor_plugin_imu ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_imu astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a sensor plugin for the perch camera
# add_library(gazebo_sensor_plugin_perch_cam
#   src/gazebo_sensor_plugin_perch_cam/gazebo_sensor_plugin_perch_cam.cc
# )
# add_dependencies(gazebo_sensor_plugin_perch_cam ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_perch_cam astrobee_gazebo
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# Create a sensor plugin for the handrail detection algorithm
# add_library(gazebo_sensor_plugin_handrail_detect
#   src/gazebo_sensor_plugin_handrail_detect/gazebo_sensor_plugin_handrail_detect.cc
# )
# add_dependencies(gazebo_sensor_plugin_handrail_detect ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_handrail_detect
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a sensor plugin for the hazard camera
# add_library(gazebo_sensor_plugin_haz_cam
#   src/gazebo_sensor_plugin_haz_cam/gazebo_sensor_plugin_haz_cam.cc
# )
# add_dependencies(gazebo_sensor_plugin_haz_cam ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_haz_cam
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a sensor plugin for the nav cam
# add_library(gazebo_sensor_plugin_nav_cam
#   src/gazebo_sensor_plugin_nav_cam/gazebo_sensor_plugin_nav_cam.cc
# )
# add_dependencies(gazebo_sensor_plugin_nav_cam ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_nav_cam
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a sensor plugin for the dock cam
# add_library(gazebo_sensor_plugin_dock_cam
#   src/gazebo_sensor_plugin_dock_cam/gazebo_sensor_plugin_dock_cam.cc
# )
# add_dependencies(gazebo_sensor_plugin_dock_cam ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_dock_cam
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a model plugin for sparse mapping
# add_library(gazebo_sensor_plugin_sparse_map
#   src/gazebo_sensor_plugin_sparse_map/gazebo_sensor_plugin_sparse_map.cc
# )
# add_dependencies(gazebo_sensor_plugin_sparse_map ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_sparse_map
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a model plugin for AR localization
# add_library(gazebo_sensor_plugin_ar_tags
#   src/gazebo_sensor_plugin_ar_tags/gazebo_sensor_plugin_ar_tags.cc
# )
# add_dependencies(gazebo_sensor_plugin_ar_tags ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_ar_tags
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a model plugin for optical flow
# add_library(gazebo_sensor_plugin_optical_flow
#   src/gazebo_sensor_plugin_optical_flow/gazebo_sensor_plugin_optical_flow.cc
# )
# add_dependencies(gazebo_sensor_plugin_optical_flow ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_sensor_plugin_optical_flow
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

# # Create a world plugin for speed
add_library(gazebo_world_plugin_speed SHARED
  src/gazebo_world_plugin_speed/gazebo_world_plugin_speed.cc
)
target_link_libraries(gazebo_world_plugin_speed astrobee_gazebo)
ament_target_dependencies(gazebo_world_plugin_speed rclcpp gazebo_dev gazebo_ros)
ament_export_libraries(gazebo_world_plugin_speed)

# # Create a system plugin for the client gui
# add_library(gazebo_system_plugin_client
#   src/gazebo_system_plugin_client/gazebo_system_plugin_client.cc
# )
# add_dependencies(gazebo_system_plugin_client ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_system_plugin_client
#   ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_export_dependencies(ament_cmake rclcpp)

ament_export_include_directories(include)


## Install
install(TARGETS
  astrobee_gazebo
  gazebo_model_plugin_eps
  gazebo_model_plugin_drag
  gazebo_model_plugin_truth
  # gazebo_model_plugin_pmc
  gazebo_model_plugin_perching_arm
  gazebo_model_plugin_flashlight
  gazebo_world_plugin_speed
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# Also install some other resources
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Mark launch files for installation
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE)


# pluginlib_export_plugin_description_file(astrobee_gazebo gazebo_astrobee_plugins.xml)

ament_package()
# endif (ENABLE_GAZEBO)
