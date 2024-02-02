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
from position import Position
from timestamped_pose import TimestampedPose
from value_plotter import ValuePlotter
from vector3d_plotter import Vector3dPlotter
import scipy.spatial.transform
import sys

# Return list of 3 lists, one each for x, y, z values in poses
def xyz_vectors_from_poses(poses):
    # TODO: Do this more efficiently
    xs = [pose.position.x for pose in poses] 
    ys = [pose.position.y for pose in poses] 
    zs = [pose.position.z for pose in poses] 
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

# Return list of graph vio poses from graph vio states.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a graph_vio_state.
def adjusted_graph_vio_poses_from_graph_vio_states(graph_vio_states, groundtruth_poses, max_diff = 0.1):
    graph_vio_state_times = np.array([state.timestamp for state in graph_vio_states])
    world_T_vio = None
    for groundtruth_pose in groundtruth_poses:
        closest_matching_graph_vio_state_index = np.argmin(np.abs(graph_vio_state_times - groundtruth_pose.timestamp))
        timestamp_diff = np.abs(graph_vio_state_times[closest_matching_graph_vio_state_index] - groundtruth_pose.timestamp)
        if timestamp_diff <= max_diff:
            closest_graph_vio_state = graph_vio_states[closest_matching_graph_vio_state_index]
            world_T_vio = groundtruth_pose * graph_vio_states[closest_matching_graph_vio_state_index].pose_with_covariance.inverse()
            break
    if not world_T_vio:
        print("Failed to find corresponding groundtruth pose to graph VIO poses")
        sys.exit(0)     
    adjusted_graph_vio_poses = []
    for graph_vio_state in graph_vio_states:
        adjusted_pose = world_T_vio * TimestampedPose(graph_vio_state.pose_with_covariance.orientation, graph_vio_state.pose_with_covariance.position, graph_vio_state.timestamp)
        adjusted_graph_vio_poses.append(TimestampedPose(adjusted_pose.orientation, adjusted_pose.position, graph_vio_state.timestamp))
    return adjusted_graph_vio_poses

# Return list of graph vio poses from graph vio state integrated velocities.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a graph_vio_state.
def absolute_poses_from_integrated_graph_vio_state_velocities(graph_vio_states, groundtruth_poses, max_diff = 0.1):
    graph_vio_state_times = np.array([state.timestamp for state in graph_vio_states])
    start_index = None
    starting_groundtruth_pose = None
    for i in range(len(groundtruth_poses)):
        closest_matching_graph_vio_state_index = np.argmin(np.abs(graph_vio_state_times - groundtruth_poses[i].timestamp))
        timestamp_diff = np.abs(graph_vio_state_times[closest_matching_graph_vio_state_index] - groundtruth_poses[i].timestamp)
        if timestamp_diff <= max_diff:
            start_index = closest_matching_graph_vio_state_index
            starting_groundtruth_pose = groundtruth_poses[i]
            break
    if not start_index:
        print("Failed to find corresponding groundtruth pose to graph VIO poses")
        sys.exit(0)  
    return integrate_velocities(graph_vio_states[start_index:], starting_groundtruth_pose)


# Integrates graph vio velocities and appends these to a starting pose to generate absolute pose estimates 
def integrate_velocities(graph_vio_states, starting_pose):
    # Calculate relative x,y,z increments using v*dt for each increment
    # Succesively add these increments to starting pose to generate future poses
    x = starting_pose.position.x
    y = starting_pose.position.y
    z = starting_pose.position.z
    integrated_poses = []
    for i in range(len(graph_vio_states)-1):
        integrated_poses.append(TimestampedPose(scipy.spatial.transform.Rotation.from_rotvec([0,0,0]), Position([x, y, z]), graph_vio_states[i].timestamp))
        dt = graph_vio_states[i+1].timestamp - graph_vio_states[i].timestamp 
        dx = dt*graph_vio_states[i].velocity_with_covariance.x
        dy = dt*graph_vio_states[i].velocity_with_covariance.y
        dz = dt*graph_vio_states[i].velocity_with_covariance.z
        x += dx
        y += dy
        z += dz
    return integrated_poses
    

# Return list of optical flow feature counts from graph vio states
def optical_flow_feature_counts_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.num_detected_of_features for graph_vio_state in graph_vio_states]

# Return list of optical flow factor counts from graph vio states
def optical_flow_factor_counts_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.num_of_factors for graph_vio_state in graph_vio_states]

# Return list of ml pose factor counts from graph loc states
def ml_pose_factor_counts_from_graph_loc_states(graph_loc_states):
    return [graph_loc_state.num_ml_pose_factors for graph_loc_state in graph_loc_states]

# Return list of ml projection factor counts from graph loc states
def ml_projection_factor_counts_from_graph_loc_states(graph_loc_states):
    return [graph_loc_state.num_ml_projection_factors for graph_loc_state in graph_loc_states]

# Return list of optimization times from states
def optimization_times_from_states(states):
    return [state.optimization_time for state in states]

# Return list of update times from states
def update_times_from_states(states):
    return [state.update_time for state in states]

# Return list of timestamped poses from graph loc states
def poses_from_graph_loc_states(graph_loc_states):
    return [TimestampedPose(graph_loc_state.pose_with_covariance.orientation, graph_loc_state.pose_with_covariance.position, graph_loc_state.timestamp)  for graph_loc_state in graph_loc_states]

def velocity_plotter_from_graph_vio_states(graph_vio_states):
    xs, ys, zs = xyz_velocity_vectors_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states) 
    return Vector3dPlotter("Graph VIO Velocity", times, xs, ys, zs, ['X', 'Y', 'Z'], marker='o')

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

def ml_pose_factor_count_plotter_from_graph_loc_states(graph_loc_states):
  counts = ml_pose_factor_counts_from_graph_loc_states(graph_loc_states)
  times = times_from_timestamped_objects(graph_loc_states) 
  return ValuePlotter("Graph Loc ML Pose Factors", times, counts, "Time (s)", "Num Factors", "ML")

def ml_projection_factor_count_plotter_from_graph_loc_states(graph_loc_states):
  counts = ml_projection_factor_counts_from_graph_loc_states(graph_loc_states)
  times = times_from_timestamped_objects(graph_loc_states) 
  return ValuePlotter("Graph Loc ML Projection Factors", times, counts, "Time (s)", "Num Factors", "ML")

def optimization_time_plotter_from_states(states):
  optimization_times = optimization_times_from_states(states)
  times = times_from_timestamped_objects(states) 
  return ValuePlotter("Optimization Times", times, optimization_times, "Time (s)", "Opt. Time (s)", "Opt. Time")

def update_time_plotter_from_states(states):
  update_times = update_times_from_states(states)
  times = times_from_timestamped_objects(states) 
  return ValuePlotter("Update Times", times, update_times, "Time (s)", "Update Time (s)", "Update Time")
