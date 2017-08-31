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

project(localization_node)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs cv_bridge image_transport rosbag ff_msgs tf2
  DEPENDS sparse_mapping
)

create_library(TARGET roslocalization
  LIBS ${SPARSE_MAPPING_LIBRARIES} ff_nodelet msg_conversions ${catkin_LIBRARIES}
  INC ${catkin_INCLUDE_DIRS}
  DEPS sparse_mapping ff_msgs
)

create_tool_targets(DIR tools
  LIBS ${SPARSE_MAPPING_LIBRARIES} roslocalization ${catkin_LIBRARIES}
  INC ${catkin_INCLUDE_DIRS}
  DEPS roslocalization
)

install_launch_files()
