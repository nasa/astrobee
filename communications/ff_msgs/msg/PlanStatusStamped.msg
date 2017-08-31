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
#
# Plan status message. Based off of PlanStatus from DDS. Note that while in
# ROS we use an unbounded array, we are actually limited to ~64 previous
# status messages in the DDS type.

# Header with timestamp
std_msgs/Header header

# Name of plan
string name

int32 point                 # Current station or segment
int32 command               # Current subcommand within station/segment or -1
ff_msgs/AckStatus status    # Status of the currently executing plan element

ff_msgs/Status[] history    # Completion status of the last 64 plan elements
