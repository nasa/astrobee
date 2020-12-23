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

cmake_minimum_required(VERSION 2.8.12) # Need get_filename_component DIRECTORY
project(planner_qp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mapper
  planner_trapezoidal
  ff_msgs
  ff_util
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
# Find OpenCV3
find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

find_package(JPS3D REQUIRED)
# find_package(GUROBI REQUIRED)
find_package(IntelMKL REQUIRED)
find_package(DecompUtil REQUIRED)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Polynomial.msg
  SolverInfo.msg
  Spline.msg
  Trajectory.msg
)

catkin_package(
  LIBRARIES
    planner_qp
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_util
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JPS3D_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(traj_opt_basic
  src/traj_opt_basic/msg_traj.cpp
  src/traj_opt_basic/polynomial_basis.cpp
  src/traj_opt_basic/trajectory.cpp
)
add_dependencies(traj_opt_basic ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_opt_basic ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(traj_opt_pro
  src/traj_opt_pro/nonlinear_polynomial.cpp
  src/traj_opt_pro/nonlinear_solver.cpp
  src/traj_opt_pro/nonlinear_trajectory.cpp
  src/traj_opt_pro/polynomial_basis.cpp
  src/traj_opt_pro/trajectory_solver.cpp
)
add_dependencies(traj_opt_pro ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_opt_pro traj_opt_basic ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(traj_opt_ros
  src/traj_opt_ros/ros_bridge.cpp
  # src/traj_opt_ros/trajectory_display.cpp
  # src/traj_opt_ros/trajectory_visual.cpp
)
add_dependencies(traj_opt_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_opt_ros traj_opt_basic ${catkin_LIBRARIES})


# Declare C++ libraries
add_library(planner_qp
  src/planner_qp.cc
)
add_dependencies(planner_qp ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_qp traj_opt_basic traj_opt_pro traj_opt_ros ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)