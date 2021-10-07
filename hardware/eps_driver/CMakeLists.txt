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
project(eps_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED)  # defines catkin_DIR
SET(catkin2_DIR "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  ff_util
  ff_hw_msgs
  i2c
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    eps_driver
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    ff_util
    ff_hw_msgs
    i2c
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(eps_driver
  src/eps_driver/eps_driver.cc
)
add_dependencies(eps_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(eps_driver ${catkin_LIBRARIES})


add_library(eps_driver_node
  src/eps_driver_node/eps_driver_node.cc
)
add_dependencies(eps_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(eps_driver_node eps_driver ${catkin_LIBRARIES})

## Declare a C++ executable: eps_driver_tool
add_executable(eps_driver_tool tools/eps_driver_tool/eps_driver_tool.cc)
add_dependencies(eps_driver_tool ${catkin_EXPORTED_TARGETS})
target_link_libraries(eps_driver_tool
  eps_driver gflags ${catkin_LIBRARIES})

## Declare a C++ executable: eps_simulator
add_executable(eps_simulator tools/eps_simulator/eps_simulator.cc)
add_dependencies(eps_simulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(eps_simulator
  eps_driver gflags ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
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

# Install C++ executables
install(TARGETS eps_driver_tool DESTINATION bin)
install(TARGETS eps_driver_tool DESTINATION share/${PROJECT_NAME})
install(TARGETS eps_simulator DESTINATION bin)
install(TARGETS eps_simulator DESTINATION share/${PROJECT_NAME})


# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(FILES
  tools/eps_aux.py
  DESTINATION share/${PROJECT_NAME})

