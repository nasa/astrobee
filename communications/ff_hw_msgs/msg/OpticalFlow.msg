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
# Message used to store and send eps state to the dds bridge

# Header with timestamp
std_msgs/Header header

float32 ground_distance  # distance to ground in meters
int16   flow_x           # x-component of optical flow in pixels
int16   flow_y           # y-component of optical flow in pixels
float32 velocity_x       # x-component of scaled optical flow in m/s
float32 velocity_y       # y-component of scaled optical flow in m/s
uint8   quality          # quality of optical flow estimate
