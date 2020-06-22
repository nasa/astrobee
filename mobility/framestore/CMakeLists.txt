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

project(framestore)

# Call catkin
catkin_package(
  CATKIN_DEPENDS
    roscpp roscpp tf2 tf2_ros geometry_msgs
)

# Create global_transforms
create_tool_targets(DIR tools
  LIBS ${catkin_LIBRARIES} ff_common msg_conversions config_reader
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_common msg_conversions config_reader tf2 tf2_ros geometry_msgs
)

# Create framestore
create_library(TARGET framestore
  LIBS ${catkin_LIBRARIES} ff_common ff_nodelet msg_conversions
  INC  ${catkin_INCLUDE_DIRS}
  DEPS ff_common msg_conversions roscpp nodelet pluginlib tf2 tf2_ros geometry_msgs
)

# Install our launch file
install_launch_files()
