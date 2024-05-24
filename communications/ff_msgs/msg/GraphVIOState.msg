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
std_msgs/Header header # header with timestamp
string child_frame_id # frame ID
# CombinedNavState estimates currently in the graph
CombinedNavStateArray combined_nav_states 
# Stats
uint32 num_detected_of_features  
uint32 optimization_iterations 
float32 optimization_time
float32 update_time # Include optimization_time and other operations to add data to graph
float32 duration 
uint32 num_factors
uint32 num_of_factors
uint32 num_depth_factors
uint32 num_states 
# Status
bool standstill
# TODO(rsoussan): Fill this in! Estimating bias expected for gnc ekf message!
bool estimating_bias # Are we busy estimating the bias?
