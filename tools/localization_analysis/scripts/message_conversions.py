#!/usr/bin/python3
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
import scipy.spatial.transform

import states


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
    position = states.Position(
        [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
    )
    return orientation, position


# Helper function to return orientation, position, and timestamp from a pose msg.
def orientation_position_timestamp_from_msg(pose_msg, bag_start_time=0):
    orientation, position = orientation_position_from_msg(pose_msg)
    timestamp = relative_timestamp(pose_msg.header.stamp, bag_start_time)
    return orientation, position, timestamp


# Create a timestamped pose with covariance from a pose msg using relative bag time.
def timestamped_pose_from_msg(pose_msg, bag_start_time=0):
    orientation, position, timestamp = orientation_position_timestamp_from_msg(
        pose_msg, bag_start_time
    )
    return states.TimestampedPose(orientation, position, timestamp)


# Create a timestamped pose with covariance from an odometry msg using relative bag time.
def timestamped_pose_from_odometry_msg(odometry_msg, bag_start_time=0):
    orientation, position = orientation_position_from_msg(
        odometry_msg.body_F_source_T_target
    )
    timestamp = relative_timestamp(odometry_msg.source_time, bag_start_time)
    return states.TimestampedPose(orientation, position, timestamp)


# Create a pose from a pose msg
def pose_from_msg(pose_msg):
    orientation, position = orientation_position_from_msg(pose_msg)
    return states.Pose(orientation, position)


# Create a pose with covariance from a pose msg
def pose_with_covariance_from_msg(pose_msg):
    orientation, position = orientation_position_from_msg(pose_msg)
    return states.PoseWithCovariance(orientation, position, pose_msg.covariance)


# Create a timestamped pose with covariance from a pose msg using relative bag time.
def timestamped_pose_with_covariance_from_msg(pose_msg, bag_start_time=0):
    orientation, position, timestamp = orientation_position_timestamp_from_msg(
        pose_msg, bag_start_time
    )
    return states.TimestampedPoseWithCovariance(
        orientation, position, pose_msgs.covariance, timestamp
    )


# Create a timestamped velocity from a velocity msg using relative bag time.
def timestamped_velocity_from_msg(velocity_msg, bag_start_time=0):
    timestamp = relative_timestamp(velocity_msg.header.stamp, bag_start_time)
    return states.TimestampedVelocity(
        velocity_msg.velocity.x,
        velocity_msg.velocity.y,
        velocity_msg.velocity.z,
        timestamp,
    )


# Create a velocity from a velocity msg.
def velocity_from_msg(velocity_msg):
    return states.Velocity(
        velocity_msg.velocity.x, velocity_msg.velocity.y, velocity_msg.velocity.z
    )


# Create an acceleration from an acceleration msg.
def acceleration_from_msg(acceleration_msg):
    return states.Acceleration(
        acceleration_msg.accel.x, acceleration_msg.accel.y, acceleration_msg.accel.z
    )


# Create an acceleration from an imu msg.
def timestamped_acceleration_from_imu_msg(imu_msg, bag_start_time=0):
    timestamp = relative_timestamp(imu_msg.header.stamp, bag_start_time)
    return states.TimestampedAcceleration(
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z,
        timestamp,
    )


# Create a timestamped velocity from a velocity msg using relative bag time.
def velocity_with_covariance_from_msg(velocity_msg, bag_start_time=0):
    return states.VelocityWithCovariance(
        velocity_msg.velocity.x,
        velocity_msg.velocity.y,
        velocity_msg.velocity.z,
        velocity_msg.covariance,
    )


# Create a depth odometry object from a msg using relative bag time.
def depth_odometry_from_msg(msg, bag_start_time=0):
    depth_odometry = states.DepthOdometry()
    depth_odometry.timestamp = relative_timestamp(msg.header.stamp, bag_start_time)
    depth_odometry.pose_with_covariance = pose_with_covariance_from_msg(
        msg.odometry.body_F_source_T_target
    )
    depth_odometry.num_features = len(msg.correspondences)
    depth_odometry.runtime = msg.runtime
    return depth_odometry


# Create a graph vio state from a msg using relative bag time.
def graph_vio_state_from_msg(msg, bag_start_time=0):
    graph_vio_state = states.GraphVIOState()
    # TODO: load all combined nav states???
    graph_vio_state.timestamp = relative_timestamp(msg.header.stamp, bag_start_time)
    latest_state = msg.combined_nav_states.combined_nav_states[-1]
    graph_vio_state.pose_with_covariance = pose_with_covariance_from_msg(
        latest_state.pose.pose
    )
    graph_vio_state.velocity_with_covariance = velocity_with_covariance_from_msg(
        latest_state.velocity
    )
    # TODO: make function for this?
    accelerometer_bias = states.AccelerometerBias(
        latest_state.imu_bias.accelerometer_bias.x,
        latest_state.imu_bias.accelerometer_bias.y,
        latest_state.imu_bias.accelerometer_bias.z,
    )
    gyro_bias = states.GyroscopeBias(
        latest_state.imu_bias.gyroscope_bias.x,
        latest_state.imu_bias.gyroscope_bias.y,
        latest_state.imu_bias.gyroscope_bias.z,
    )
    # TODO: load covariance?
    graph_vio_state.imu_bias_with_covariance = states.ImuBias(
        accelerometer_bias, gyro_bias
    )
    graph_vio_state.num_detected_of_features = msg.num_detected_of_features
    graph_vio_state.num_of_factors = msg.num_of_factors
    graph_vio_state.num_depth_factors = msg.num_depth_factors
    graph_vio_state.num_states = msg.num_states
    graph_vio_state.standstill = int(msg.standstill)
    graph_vio_state.optimization_iterations = msg.optimization_iterations
    graph_vio_state.optimization_time = msg.optimization_time
    graph_vio_state.update_time = msg.update_time
    graph_vio_state.duration = msg.duration
    return graph_vio_state


# Create a graph loc state from a msg using relative bag time.
def graph_loc_state_from_msg(msg, bag_start_time=0):
    graph_loc_state = states.GraphLocState()
    graph_loc_state.timestamp = relative_timestamp(msg.header.stamp, bag_start_time)
    graph_loc_state.pose_with_covariance = pose_with_covariance_from_msg(msg.pose)
    graph_loc_state.num_detected_ar_features = msg.num_detected_ar_features
    graph_loc_state.num_detected_ml_features = msg.num_detected_ml_features
    graph_loc_state.optimization_iterations = msg.optimization_iterations
    graph_loc_state.optimization_time = msg.optimization_time
    graph_loc_state.update_time = msg.update_time
    graph_loc_state.duration = msg.duration
    graph_loc_state.num_factors = msg.num_factors
    graph_loc_state.num_ml_projection_factors = msg.num_ml_projection_factors
    graph_loc_state.num_ml_pose_factors = msg.num_ml_pose_factors
    graph_loc_state.num_states = msg.num_states
    return graph_loc_state


# Create a extrapolated loc state from a msg using relative bag time.
def extrapolated_loc_state_from_msg(msg, bag_start_time=0):
    extrapolated_loc_state = states.ExtrapolatedLocState()
    extrapolated_loc_state.timestamp = relative_timestamp(
        msg.header.stamp, bag_start_time
    )
    extrapolated_loc_state.pose = pose_from_msg(msg)
    extrapolated_loc_state.velocity = velocity_from_msg(msg)
    extrapolated_loc_state.acceleration = acceleration_from_msg(msg)
    return extrapolated_loc_state
