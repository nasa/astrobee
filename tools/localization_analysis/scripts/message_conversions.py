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

import numpy as np
from accelerometer_bias import AccelerometerBias
from graph_vio_state import GraphVIOState
from gyroscope_bias import GyroscopeBias
from imu_bias import ImuBias
from pose_with_covariance import PoseWithCovariance
from timestamped_pose import TimestampedPose
from timestamped_pose_with_covariance import TimestampedPoseWithCovariance
import timestamped_velocity 
from velocity_with_covariance import VelocityWithCovariance
import scipy.spatial.transform

# Subtract the bag start time from the timestamp
# to make start time relative to the bag start time 
def relative_timestamp(timestamp, bag_start_time):
    return timestamp.secs + 1e-9 * timestamp.nsecs - bag_start_time

# Helper function to return orientation and position from a pose msg. 
def orientation_position_from_msg(pose_msg):
    orientation = scipy.spatial.transform.Rotation.from_quat(
        [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]
    )
    return orientation, [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]


# Helper function to return orientation, position, and timestamp from a pose msg. 
def orientation_position_timestamp_from_msg(pose_msg, bag_start_time=0):
    orientation, position = orientation_position_from_msg(pose_msg)
    timestamp = relative_timestamp(pose_msg.header.stamp, bag_start_time)
    return orientation, position, timestamp



# Create a timestamped pose with covariance from a pose msg using relative bag time. 
def timestamped_pose_from_msg(pose_msg, bag_start_time=0):
    orientation, position, timestamp = orientation_position_timestamp_from_msg(pose_msg, bag_start_time)
    return TimestampedPose(orientation, position, timestamp)

# Create a pose with covariance from a pose msg 
def pose_with_covariance_from_msg(pose_msg):
    orientation, position = orientation_position_from_msg(pose_msg)
    return PoseWithCovariance(orientation, position, pose_msg.covariance)


# Create a timestamped pose with covariance from a pose msg using relative bag time. 
def timestamped_pose_with_covariance_from_msg(pose_msg, bag_start_time=0):
    orientation, position, timestamp = orientation_position_timestamp_from_msg(pose_msg, bag_start_time)
    return TimestampedPoseWithCovariance(orientation, position, pose_msgs.covariance, timestamp)

# Create a timestamped velocity from a velocity msg using relative bag time. 
def timestamped_velocity_from_msg(velocity_msg, bag_start_time=0):
    timestamp = relative_timestamp(velocity_msg.header.stamp, bag_start_time)
    return TimestampedVelocity(velocity_msg.velocity.x, velocity_msg.velocity.y, velocity_msg.velocity.z, timestamp)

# Create a timestamped velocity from a velocity msg using relative bag time. 
def velocity_with_covariance_from_msg(velocity_msg, bag_start_time=0):
    return VelocityWithCovariance(velocity_msg.velocity.x, velocity_msg.velocity.y, velocity_msg.velocity.z, velocity_msg.covariance)

# Create a graph vio state from a msg using relative bag time. 
def graph_vio_state_from_msg(msg, bag_start_time=0):
    graph_vio_state = GraphVIOState()
    # TODO: load all combined nav states???
    graph_vio_state.timestamp = relative_timestamp(msg.header.stamp, bag_start_time)
    latest_state = msg.combined_nav_states.combined_nav_states[-1]
    graph_vio_state.pose_with_covariance = pose_with_covariance_from_msg(latest_state.pose)
    graph_vio_state.velocity_with_covariance = velocity_with_covariance_from_msg(latest_state.velocity)
    # TODO: make function for this?
    accelerometer_bias = AccelerometerBias(latest_state.imu_bias.accelerometer_bias.x, latest_state.imu_bias.accelerometer_bias.y, latest_state.imu_bias.accelerometer_bias.z) 
    gyro_bias = GyroscopeBias(latest_state.imu_bias.gyroscope_bias.x, latest_state.imu_bias.gyroscope_bias.y, latest_state.imu_bias.gyroscope_bias.z) 
    # TODO: load covariance?
    graph_vio_state.imu_bias_with_covariance = ImuBias(accelerometer_bias, gyro_bias)
    graph_vio_state.num_detected_of_features = msg.num_detected_of_features
    graph_vio_state.num_of_factors = msg.num_of_factors
    return graph_vio_state
