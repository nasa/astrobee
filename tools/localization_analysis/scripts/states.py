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

# States used for plotting/analysis tools


# Acceleration object
class Acceleration(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


# Accelerometer Bias object
class AccelerometerBias(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


# DepthOdometry object containing information from a DepthOdometry Msg
class DepthOdometry:
    def __init__(self):
        self.timestamp = None
        self.pose_with_covariance = None
        self.num_features = None
        self.runtime = None


# ExtrapolatedLocState object containing information from a Ekf Msg
class ExtrapolatedLocState:
    def __init__(self):
        self.timestamp = None
        self.pose = None
        self.velocity = None
        self.acceleration = None


# GraphLocState object containing information from a GraphLocState Msg
class GraphLocState:
    def __init__(self):
        self.timestamp = None
        self.pose_with_covariance = None
        self.num_detected_ar_features = None
        self.num_detected_ml_features = None
        self.optimization_iterations = None
        self.optimization_time = None
        self.update_time = None
        self.num_factors = None
        self.num_ml_projection_factors = None
        self.num_ml_pose_factors = None
        self.num_states = None
        self.duration = None


# GraphVIOState object containing information from a GraphVIOState Msg
class GraphVIOState:
    def __init__(self):
        self.timestamp = None
        self.pose_with_covariance = None
        self.velocity_with_covariance = None
        self.imu_bias_with_covariance = None
        self.num_detected_of_features = None
        self.num_of_factors = None
        self.num_depth_factors = None
        self.num_states = None
        self.optimization_iterations = None
        self.optimization_time = None
        self.update_time = None
        self.duration = None
        self.standstill = None


# Gyroscope Bias object
class GyroscopeBias(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class ImuBias(object):
    def __init__(self, accelerometer_bias, gyroscope_bias):
        self.accelerometer_bias = accelerometer_bias
        self.gyroscope_bias = gyroscope_bias


# Class that contains an imu bias and covariance.
class ImuBiasWithCovariance(ImuBias):
    def __init__(self, accelerometer_bias, gyroscope_bias, covariance):
        super(ImuBiasWithCovariance, self).__init__(accelerometer_bias, gyroscope_bias)
        self.covariance = covariance


# Position class that contains x, y, and z values.
class Position(np.ndarray):
    def __new__(cls, position_vector):
        self = np.asarray(position_vector).view(cls)
        self.x = position_vector[0]
        self.y = position_vector[1]
        self.z = position_vector[2]
        return self


# Pose class that contains an orientation and position and supports pose multiplication.
class Pose(object):
    def __init__(self, orientation, position):
        self.orientation = orientation
        self.position = position

    # Right multiply the pose by pose_b and return the resulting pose.
    def __mul__(self, pose_b):
        new_orientation = self.orientation * pose_b.orientation
        new_position = Position(self.orientation.apply(pose_b.position) + self.position)
        return Pose(new_orientation, new_position)

    # Invert the pose
    def inverse(self):
        new_orientation = self.orientation.inv()
        new_position = Position(-1.0 * new_orientation.apply(self.position))
        return Pose(new_orientation, new_position)

    # Returns the orientation as ZYX euler angles (YPR).
    def euler_angles(self):
        return self.orientation.as_euler("ZYX", degrees=True)

    # Returns the position.
    def position(self):
        return self.position


# Class that contains a pose and covariance.
class PoseWithCovariance(Pose):
    def __init__(self, orientation, position, covariance):
        super(PoseWithCovariance, self).__init__(orientation, position)
        self.covariance = covariance


# Velocity object
class Velocity(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


# Class that contains a velocity and covariance.
class VelocityWithCovariance(Velocity):
    def __init__(self, x, y, z, covariance):
        super(VelocityWithCovariance, self).__init__(x, y, z)
        self.covariance = covariance


# Class that contains a timestamped acceleration
class TimestampedAcceleration(Acceleration):
    def __init__(self, x, y, z, timestamp):
        super(TimestampedAcceleration, self).__init__(x, y, z)
        self.timestamp = timestamp


# Class that contains a timestamped imu_bias and covariance.
class TimestampedImuBiasWithCovariance(ImuBiasWithCovariance):
    def __init__(self, imu_bias, covariance, timestamp):
        self.imu_bias = imu_bias
        self.covariance = covariance
        self.timestamp = timestamp


# Class that contains a timestamped pose.
class TimestampedPose(Pose):
    def __init__(self, orientation, position, timestamp):
        super(TimestampedPose, self).__init__(orientation, position)
        self.timestamp = timestamp


# Class that contains a timestamped pose and covariance.
class TimestampedPoseWithCovariance(PoseWithCovariance):
    def __init__(self, orientation, position, covariance, timestamp):
        super(TimestampedPoseWithCovariance, self).__init__(orientation, position)
        self.timestamp = timestamp


# Class that contains a timestamped velocity and covariance.
class TimestampedVelocity(Velocity):
    def __init__(self, x, y, z, timestamp):
        super(TimestampedVelocity, self).__init__(x, y, z)
        self.timestamp = timestamp


# Class that contains a timestamped velocity and covariance.
class TimestampedVelocityWithCovariance(VelocityWithCovariance):
    def __init__(self, velocity, covariance, timestamp):
        self.velocity = velocity
        self.covariance = covariance
        self.timestamp = timestamp
