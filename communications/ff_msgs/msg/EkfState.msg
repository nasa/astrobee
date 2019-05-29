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
# An observation of a handrail from a depth image.

std_msgs/Header header # header with timestamp
string child_frame_id # frame ID

geometry_msgs/Pose pose # robot body pose

# m/s
geometry_msgs/Vector3 velocity # the body velocity

# rad/s
geometry_msgs/Vector3 omega # body rotational velocity
geometry_msgs/Vector3 gyro_bias # estimated gyro bias

# m/s/s
geometry_msgs/Vector3 accel # acceleration in body frame
geometry_msgs/Vector3 accel_bias # estimated accel bias

# Filter Health

# covariance diagonal. 1-3 orientation, 4-6 gyro bias, 7-9 velocity, 10-12 accel bias, 13-15 position
float32[15] cov_diag
# confidence in EKF. 0 is good, 1 is a bit confused, 2 is lost
uint8 confidence
uint8 CONFIDENCE_GOOD = 0	# Tracking well
uint8 CONFIDENCE_POOR = 1	# Tracking poorly
uint8 CONFIDENCE_LOST = 2	# We are lost

uint8 aug_state_enum # bitmask of augmented states intialized

# status byte sent by GNC
uint8 status
uint8 STATUS_INVALID = 255	# invalid

# optical flow features this frame (0 if no update)
uint8 of_count
# ml features this frame (0 if no update)
uint8 ml_count

# Global Handrail Pose
geometry_msgs/Pose hr_global_pose

# mahalanobis distances for features
float32[50] ml_mahal_dists

# Are we busy estimating the bias?
bool estimating_bias
