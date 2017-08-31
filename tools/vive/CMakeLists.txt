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

project(vive)

# Find OpenVR installed in the prescribed directory
set(OPENVR "$ENV{HOME}/libraries/openvr")
find_library(OPENVR_LIBRARIES openvr_api "${OPENVR}/lib/linux64" NO_DEFAULT_PATH)
find_path(OPENVR_INCLUDE_DIRS openvr.h "${OPENVR}/headers" NO_DEFAULT_PATH)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    ff_msgs
)

create_tool_targets(DIR tools
  LIBS ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OPENVR_LIBRARIES} common config_reader msg_conversions
  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OPENVR_INCLUDE_DIRS} 
)

install_launch_files()
