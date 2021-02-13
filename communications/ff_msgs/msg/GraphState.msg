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
# State Estimates
geometry_msgs/Pose pose # world_T_body 
geometry_msgs/Vector3 velocity # body velocity
geometry_msgs/Vector3 gyro_bias # estimated gyro bias
geometry_msgs/Vector3 accel_bias # estimated accel bias
# Covariances/Confidences
# covariance diagonal. 1-3 orientation, 4-6 gyro bias, 7-9 velocity, 10-12 accel bias, 13-15 position
float32[15] cov_diag
# confidence in estimate. 0 is good, 1 is a bit confused, 2 is lost
uint8 confidence
uint8 CONFIDENCE_GOOD = 0	# Tracking well
uint8 CONFIDENCE_POOR = 1	# Tracking poorly
uint8 CONFIDENCE_LOST = 2	# We are lost
# Stats
uint8 of_count # optical flow feature count  
uint8 ml_count # ml feature count 
uint32 iterations # Optimization iterations
float32 optimization_time
float32 update_time # Include optimization_time and other operations to add data to graph
float32 callbacks_time # Includes processing msgs and their callbacks
float32 nodelet_runtime # Total runtime of nodelet iteration.  Includes update and callback time
uint32 num_factors
uint32 num_states
# Status
bool standstill
bool estimating_bias # Are we busy estimating the bias?
