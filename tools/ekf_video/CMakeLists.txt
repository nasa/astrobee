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

project(ekf_video)

catkin_package(
  CATKIN_DEPENDS roscpp
)

# copied this from qp planner
find_package(catkin COMPONENTS rviz QUIET)
if(rviz_QT_VERSION)
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Gui)

  create_library(TARGET ekf_video
    LIBS ekf_bag ${FFMPEG_LIBRARIES} ${catkin_LIBRARIES} ${Qt5Gui_LIBRARIES}
    INC ${catkin_INCLUDE_DIRS} ${FFMPEG_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS}
  )

  create_tool_targets(DIR tools
    LIBS ekf_video
    INC ${catkin_INCLUDE_DIRS}
  )
endif(rviz_QT_VERSION)
