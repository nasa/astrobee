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
# CameraState message, *MUST* be kept in sync with camera portion of
# rapid::ext::astrobee::TelemetryState

# nav_cam, dock_cam, etc.
string camera_name

# streaming to ground
bool streaming

# image width
uint16 stream_width
# image height
uint16 stream_height
# Rate in Hz
float32 stream_rate

# recording to disk
bool recording

# image width
uint16 record_width
# image height
uint16 record_height
# Rate in Hz
float32 record_rate

# only for sci cam
float32 bandwidth
