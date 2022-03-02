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

import poses
import vector3ds


class LocStates(poses.Poses):
    def __init__(self, loc_type, topic):
        super(LocStates, self).__init__(loc_type, topic)
        self.num_detected_of_features = []
        self.num_detected_ml_features = []
        self.num_of_factors = []
        self.num_ml_projection_factors = []
        self.accelerations = vector3ds.Vector3ds()
        self.velocities = vector3ds.Vector3ds()
        self.angular_velocities = vector3ds.Vector3ds()
        self.accelerometer_biases = vector3ds.Vector3ds()
        self.gyro_biases = vector3ds.Vector3ds()
        self.position_covariances = vector3ds.Vector3ds()
        self.orientation_covariances = vector3ds.Vector3ds()
        self.velocity_covariances = vector3ds.Vector3ds()
        self.accelerometer_bias_covariances = vector3ds.Vector3ds()
        self.gyro_bias_covariances = vector3ds.Vector3ds()

    def add_loc_state(self, msg, bag_start_time=0):
        self.add_pose_msg(msg.pose, msg.header.stamp, bag_start_time)
        if hasattr(msg, "num_detected_of_features"):
            self.num_detected_of_features.append(msg.num_detected_of_features)
        if hasattr(msg, "num_detected_ml_features"):
            self.num_detected_ml_features.append(msg.num_detected_ml_features)
        if hasattr(msg, "num_of_factors"):
            self.num_of_factors.append(msg.num_of_factors)
        if hasattr(msg, "num_ml_projection_factors"):
            self.num_ml_projection_factors.append(msg.num_ml_projection_factors)
        if hasattr(msg, "accel"):
            self.accelerations.add_vector3d(msg.accel)
        self.velocities.add_vector3d(msg.velocity)
        if hasattr(msg, "omega"):
            self.angular_velocities.add_vector3d(msg.omega)
        self.accelerometer_biases.add_vector3d(msg.accel_bias)
        self.gyro_biases.add_vector3d(msg.gyro_bias)
        # See GraphState.msg or EkfState.msg for cov_diag offsets
        self.position_covariances.add(
            msg.cov_diag[12], msg.cov_diag[13], msg.cov_diag[14]
        )
        self.orientation_covariances.add(
            msg.cov_diag[0], msg.cov_diag[1], msg.cov_diag[2]
        )
        self.velocity_covariances.add(msg.cov_diag[6], msg.cov_diag[7], msg.cov_diag[8])
        self.accelerometer_bias_covariances.add(
            msg.cov_diag[9], msg.cov_diag[10], msg.cov_diag[11]
        )
        self.gyro_bias_covariances.add(
            msg.cov_diag[3], msg.cov_diag[4], msg.cov_diag[5]
        )
