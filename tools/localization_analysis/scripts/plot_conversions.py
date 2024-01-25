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
from value_plotter import ValuePlotter
from vector3d_plotter import Vector3dPlotter
import scipy.spatial.transform

# Return list of 3 lists, one each for x, y, z values in poses
def xyz_vectors_from_poses(poses):
    # TODO: Do this more efficiently
    xs = [pose.position[0] for pose in poses] 
    ys = [pose.position[1] for pose in poses] 
    zs = [pose.position[2] for pose in poses] 
    return [xs, ys, zs]

# Return list of 3 lists, one each for y, p, r values in poses
def ypr_vectors_from_poses(poses):
    # TODO: Do this more efficiently
    ys = [euler_angles[0] for euler_angles in (pose.euler_angles() for pose in poses)]
    ps = [euler_angles[1] for euler_angles in (pose.euler_angles() for pose in poses)]
    rs = [euler_angles[2] for euler_angles in (pose.euler_angles() for pose in poses)]
    return [ys, ps, rs]

# Return list of 3 lists, one each for x, y, z values in velocities 
def xyz_velocity_vectors_from_graph_vio_states(graph_vio_states):
    # TODO: Do this more efficiently
    xs = [state.velocity_with_covariance.x for state in graph_vio_states] 
    ys = [state.velocity_with_covariance.y for state in graph_vio_states] 
    zs = [state.velocity_with_covariance.z for state in graph_vio_states] 
    return [xs, ys, zs]

# Return list of 3 lists, one each for x, y, z values in IMU accelerometer bias 
def xyz_accel_bias_vectors_from_graph_vio_states(graph_vio_states):
    # TODO: Do this more efficiently
    xs = [state.imu_bias_with_covariance.accelerometer_bias.x for state in graph_vio_states] 
    ys = [state.imu_bias_with_covariance.accelerometer_bias.y for state in graph_vio_states] 
    zs = [state.imu_bias_with_covariance.accelerometer_bias.z for state in graph_vio_states] 
    return [xs, ys, zs]

# Return list of 3 lists, one each for x, y, z values in IMU gyro bias 
def xyz_gyro_bias_vectors_from_graph_vio_states(graph_vio_states):
    # TODO: Do this more efficiently
    xs = [state.imu_bias_with_covariance.gyroscope_bias.x for state in graph_vio_states] 
    ys = [state.imu_bias_with_covariance.gyroscope_bias.y for state in graph_vio_states] 
    zs = [state.imu_bias_with_covariance.gyroscope_bias.z for state in graph_vio_states] 
    return [xs, ys, zs]


# Return list of times for given timestamped objects 
def times_from_timestamped_objects(timestamped_objects):
    return [obj.timestamp for obj in timestamped_objects]

# Return list of diagonal position covariance norms from poses 
def diagonal_position_covariance_norms_from_poses(poses):
    # Assumes covariance organized as orientation then position
    # RR RP
    # RP PP
    # Thus position diagonals occur at (3, 3), (4, 4), (5, 5) for the 6x6 matrix.
    # The msg stores covariances as an array with 36 values (from 0-35) where (x, y) occurs at 
    # x + 6*y. So (3, 3) -> 21, (4, 4) -> 28, (5, 5) -> 35
    return [np.linalg.norm(pose.covariance[21], pose.covariance[28], pose.covariance[35]) for pose in poses]

# Return list of timestamped poses with covariances from graph vio states
def timestamped_poses_with_covariance_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.timestamped_pose_with_covariance() for graph_vio_state in graph_vio_states]

# Return list of timestamped velocities with covariances from graph vio states
def timestamped_velocities_with_covariance_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.timestamped_velocity_with_covariance() for graph_vio_state in graph_vio_states]

# Return list of timestamped imu bias with covariances from graph vio states
def timestamped_imu_biases_with_covariance_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.timestamped_imu_bias_with_covariance() for graph_vio_state in graph_vio_states]

# Return list of optical flow feature counts from graph vio states
def optical_flow_feature_counts_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.num_detected_of_features for graph_vio_state in graph_vio_states]

# Return list of optical flow factor counts from graph vio states
def optical_flow_factor_counts_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.num_of_factors for graph_vio_state in graph_vio_states]

def velocity_plotter_from_graph_vio_states(graph_vio_states):
    xs, ys, zs = xyz_velocity_vectors_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states) 
    return Vector3dPlotter("Graph VIO Velocity", times, xs, ys, zs, ['X', 'Y', 'Z'])

def accel_bias_plotter_from_graph_vio_states(graph_vio_states):
   xs, ys, zs = xyz_accel_bias_vectors_from_graph_vio_states(graph_vio_states)
   times = times_from_timestamped_objects(graph_vio_states) 
   return Vector3dPlotter("Graph VIO Accel. Bias", times, xs, ys, zs, ['X', 'Y', 'Z'])

def gyro_bias_plotter_from_graph_vio_states(graph_vio_states):
  xs, ys, zs = xyz_gyro_bias_vectors_from_graph_vio_states(graph_vio_states)
  times = times_from_timestamped_objects(graph_vio_states) 
  return Vector3dPlotter("Graph VIO Gyro Bias", times, xs, ys, zs, ['X', 'Y', 'Z'])

def optical_flow_feature_count_plotter_from_graph_vio_states(graph_vio_states):
  counts = optical_flow_feature_counts_from_graph_vio_states(graph_vio_states)
  times = times_from_timestamped_objects(graph_vio_states) 
  return ValuePlotter("Graph VIO OF Counts", times, counts, "Time (s)", "Num Features", "OF")

def optical_flow_factor_count_plotter_from_graph_vio_states(graph_vio_states):
  counts = optical_flow_factor_counts_from_graph_vio_states(graph_vio_states)
  times = times_from_timestamped_objects(graph_vio_states) 
  return ValuePlotter("Graph VIO OF Factors", times, counts, "Time (s)", "Num Factors", "OF")
