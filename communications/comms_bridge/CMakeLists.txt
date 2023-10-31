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
project(comms_bridge)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  message_generation
  message_runtime
  image_transport
  std_msgs
  sensor_msgs
  ff_msgs
  topic_tools
)

catkin_package(
  LIBRARIES src
  CATKIN_DEPENDS
  message_runtime
  std_msgs
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
add_library(comms_bridge_sub
  src/bridge_subscriber.cpp
  src/ros_dds_bridge_subscriber.cpp
)
add_dependencies(comms_bridge_sub ${catkin_EXPORTED_TARGETS})
target_link_libraries(comms_bridge_sub
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_DL_LIBS} )


add_library(comms_bridge_pub
  src/bridge_publisher.cpp
  src/dds_ros_bridge_publisher.cpp
)
add_dependencies(comms_bridge_pub ${catkin_EXPORTED_TARGETS} )
target_link_libraries(comms_bridge_pub
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_DL_LIBS} )


#############
## Install ##
#############
# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}_sub ${PROJECT_NAME}_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
