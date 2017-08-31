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

project(cpu_monitor)

catkin_package(
  LIBRARIES cpu_monitor
  CATKIN_DEPENDS roscpp roslib nodelet ff_hw_msgs ff_msgs std_msgs
)

create_library(TARGET cpu_monitor
  LIBS ${catkin_LIBRARIES} config_reader ff_nodelet i2c
  INC ${catkin_INCLUDES}
  DEPS ff_msgs ff_hw_msgs
)

install_launch_files()
