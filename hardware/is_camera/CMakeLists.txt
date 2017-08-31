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

project(is_camera)

# Need to find libv4l2
find_path(V4L2_INCLUDE_DIR libv4l2.h linux/videodev2.h
  HINTS /usr ${V4L2_ROOT_DIR})
find_library(V4L2_LIBRARY NAMES libv4l2 v4l2
  HINTS /usr/lib ${V4L2_ROOT_DIR}/lib)
find_package_handle_standard_args(V4L2 DEFAULT_MSG
  V4L2_INCLUDE_DIR V4L2_LIBRARY)

catkin_package(
  LIBRARIES is_camera
  CATKIN_DEPENDS roscpp image_transport nodelet
  )

create_library(TARGET is_camera
  LIBS ${catkin_LIBRARIES} ${V4L2_LIBRARY} config_reader ff_nodelet
  INC ${catkin_INCLUDE_DIRS} ${V4L2_INCLUDE_DIR}
  DEPS ff_msgs)

create_tool_targets(DIR tools
  LIBS is_camera
  )

# I'm not sure who installed package.xml for us, but we should do the same for
# nodelet_plugins.xml
install(FILES nodelet_plugins.xml DESTINATION share/is_camera)

install_launch_files()
