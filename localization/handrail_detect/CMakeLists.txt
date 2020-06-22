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

project(handrail_detect)

if (USE_ROS)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tf2 visualization_msgs roscpp ff_msgs nodelet
  DEPENDS ${PROJECT_NAME}
)

set(CMAKE_CXX_FLAGS_HACKY "${CMAKE_CXX_FLAGS} -Wno-return-type")

create_library(TARGET ${PROJECT_NAME}
  LIBS ${catkin_LIBRARIES} ${OROCOS_KDL_LIBRARIES} ff_common config_reader msg_conversions ff_nodelet
  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPS ff_msgs
)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_HACKY})

create_tool_targets(DIR tools
  LIBS ${PROJECT_NAME}
)

install_launch_files()

endif (USE_ROS)
