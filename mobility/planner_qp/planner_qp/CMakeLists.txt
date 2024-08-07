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
project(planner_qp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wno-deprecated-declarations  -Werror=return-type")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../cmake")


# Find OpenCV
find_package(OpenCV 4.0 REQUIRED)

find_package(JPS3D REQUIRED)
find_package(DecompUtil REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)


## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  actionlib
  tf
  ff_util
  choreographer
  traj_opt_basic
  traj_opt_pro
  traj_opt_ros
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
    choreographer
    traj_opt_basic
    traj_opt_pro
    traj_opt_ros
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

add_library(planner_qp
  src/planner_qp.cc
)
add_dependencies(planner_qp ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_qp ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ executable: sfc_server
add_executable(sfc_server test/sfc_server.cc)
add_dependencies(sfc_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(sfc_server
  planner_qp gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install C++ executables
install(TARGETS sfc_server DESTINATION bin)
install(CODE "execute_process(
  COMMAND ln -s ../../bin/sfc_server share/${PROJECT_NAME}
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
  OUTPUT_QUIET
  ERROR_QUIET
  )")

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

