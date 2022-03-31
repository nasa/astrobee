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
project(is_camera)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  image_transport
  nodelet
  ff_util
  config_reader
)

# Need to find libv4l2
find_path(V4L2_INCLUDE_DIR libv4l2.h linux/videodev2.h
  HINTS /usr ${V4L2_ROOT_DIR})
find_library(V4L2_LIBRARY NAMES libv4l2 v4l2
  HINTS /usr/lib ${V4L2_ROOT_DIR}/lib)
find_package_handle_standard_args(V4L2 DEFAULT_MSG
  V4L2_INCLUDE_DIR V4L2_LIBRARY)


# Find OpenCV
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV331 REQUIRED)

catkin_package(
  LIBRARIES is_camera
  CATKIN_DEPENDS
    roscpp
    image_transport
    nodelet
    ff_util
    config_reader
  )

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(is_camera
  src/camera.cc
)
add_dependencies(is_camera ${catkin_EXPORTED_TARGETS})
target_link_libraries(is_camera ${V4L2_LIBRARY} ${catkin_LIBRARIES})

# Declare C++ libraries
add_library(debayer
  src/debayer.cc
)
add_dependencies(debayer ${catkin_EXPORTED_TARGETS})
target_link_libraries(debayer ${V4L2_LIBRARY} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark libraries for installation
install(TARGETS debayer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
