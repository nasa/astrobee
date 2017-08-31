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

project(lk_optical_flow)

if (USE_ROS)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp cv_bridge image_transport nodelet
  DEPENDS lk_optical_flow
)

create_library(TARGET lk_optical_flow
  LIBS ff_nodelet camera config_reader ${catkin_LIBRARIES}
  INC ${catkin_INCLUDE_DIRS}
  DEPS ff_msgs config_reader camera
)

install_launch_files()

endif (USE_ROS)
