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

project(disk_monitor)

catkin_package(
  LIBRARIES disk_monitor
  DEPENDS roscpp ff_msgs nodelet
)

create_library(TARGET disk_monitor
  LIBS ${catkin_LIBRARIES} ${GLOG_LIBRARIES} config_reader ff_nodelet
  INC ${catkin_INCLUDES} ${GLOG_INCLUDE_DIRS}
  DEPS ff_msgs config_reader 
)

create_test_targets(DIR test
  LIBS disk_monitor)

install_launch_files()
