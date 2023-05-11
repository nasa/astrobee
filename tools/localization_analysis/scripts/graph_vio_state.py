#!/usr/bin/python
#
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

import pose
import timestamped_imu_bias_with_covariance
import timestamped_pose_with_covariance
import timestamped_velocity_with_covariance
import velocity 

# Object containing information from a GraphVIOState Msg
class GraphVIOState(object):
    def __init__(self, msg, bag_start_time):
        #TODO: update these functions to conver x_with_covariance_from_msg!!
        self.pose_with_covariance = message_conversions.pose_from_msg(msg.pose, bag_start_time)
        self.velocity_with_covariance = message_conversions.velocity_from_msg(msg.velocity, bag_start_time)
        self.imu_bias_with_covariance = ImuBias(msg.accel_bias, msg.gyro_bias)
        self.num_detected_of_features = msg.num_detected_of_features
        self.num_of_factors = msg.num_of_factors
        self.timestamp = msg.header.stamp

    def timestamped_pose_with_covariance():
        return TimestampedPoseWithCovariance(pose_with_covariance.pose, pose_with_covariance.covariance, timestamp)

    def timestamped_velocity_with_covariance():
        return TimestampedVelocityWithCovariance(velocity_with_covariance.velocity, velocity_with_covariance.covariance, timestamp)

    def timestamped_imu_bias_with_covariance():
        return TimestampedImuBiasWithCovariance(imu_bias_with_covariance.imu_bias, imu_bias_with_covariance.covariance, timestamp)
