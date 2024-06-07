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

import sys

import numpy as np
import scipy.spatial.transform

import plotters
import states


# Return list of 3 lists, one each for x, y, z values in poses
def xyz_vectors_from_poses(poses):
    xs = [pose.position.x for pose in poses]
    ys = [pose.position.y for pose in poses]
    zs = [pose.position.z for pose in poses]
    return [xs, ys, zs]


# Return list of 3 lists, one each for y, p, r values in poses
def ypr_vectors_from_poses(poses):
    ys = [euler_angles[0] for euler_angles in (pose.euler_angles() for pose in poses)]
    ps = [euler_angles[1] for euler_angles in (pose.euler_angles() for pose in poses)]
    rs = [euler_angles[2] for euler_angles in (pose.euler_angles() for pose in poses)]
    return [ys, ps, rs]


# Return list of 3 lists, one each for x, y, z values in velocities
def xyz_velocity_vectors_from_graph_vio_states(graph_vio_states):
    xs = [state.velocity_with_covariance.x for state in graph_vio_states]
    ys = [state.velocity_with_covariance.y for state in graph_vio_states]
    zs = [state.velocity_with_covariance.z for state in graph_vio_states]
    return [xs, ys, zs]


# Return list of 3 lists, one each for x, y, z values in velocities
def xyz_velocity_vectors_from_extrapolated_loc_states(extrapolated_loc_states):
    xs = [state.velocity.x for state in extrapolated_loc_states]
    ys = [state.velocity.y for state in extrapolated_loc_states]
    zs = [state.velocity.z for state in extrapolated_loc_states]
    return [xs, ys, zs]


# Return list of 3 lists, one each for x, y, z values in accelerations
def xyz_acceleration_vectors_from_extrapolated_loc_states(extrapolated_loc_states):
    xs = [state.acceleration.x for state in extrapolated_loc_states]
    ys = [state.acceleration.y for state in extrapolated_loc_states]
    zs = [state.acceleration.z for state in extrapolated_loc_states]
    return [xs, ys, zs]


# Return list of 3 lists, one each for x, y, z values in accelerations
def xyz_acceleration_vectors_from_imu_accelerations(imu_accelerations):
    xs = [acceleration.x for acceleration in imu_accelerations]
    ys = [acceleration.y for acceleration in imu_accelerations]
    zs = [acceleration.z for acceleration in imu_accelerations]
    return [xs, ys, zs]

    return [xs, ys, zs]


# Return list of 3 lists, one each for x, y, z values in IMU accelerometer bias
def xyz_accel_bias_vectors_from_graph_vio_states(graph_vio_states):
    xs = [
        state.imu_bias_with_covariance.accelerometer_bias.x
        for state in graph_vio_states
    ]
    ys = [
        state.imu_bias_with_covariance.accelerometer_bias.y
        for state in graph_vio_states
    ]
    zs = [
        state.imu_bias_with_covariance.accelerometer_bias.z
        for state in graph_vio_states
    ]
    return [xs, ys, zs]


# Return list of 3 lists, one each for x, y, z values in IMU gyro bias
def xyz_gyro_bias_vectors_from_graph_vio_states(graph_vio_states):
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
    return [
        np.linalg.norm(pose.covariance[21], pose.covariance[28], pose.covariance[35])
        for pose in poses
    ]


# Return list of timestamped poses with covariances from graph vio states
def timestamped_poses_with_covariance_from_graph_vio_states(graph_vio_states):
    return [
        graph_vio_state.timestamped_pose_with_covariance()
        for graph_vio_state in graph_vio_states
    ]


# Return list of timestamped velocities with covariances from graph vio states
def timestamped_velocities_with_covariance_from_graph_vio_states(graph_vio_states):
    return [
        graph_vio_state.timestamped_velocity_with_covariance()
        for graph_vio_state in graph_vio_states
    ]


# Return list of timestamped imu bias with covariances from graph vio states
def timestamped_imu_biases_with_covariance_from_graph_vio_states(graph_vio_states):
    return [
        graph_vio_state.timestamped_imu_bias_with_covariance()
        for graph_vio_state in graph_vio_states
    ]


# Return list of graph vio poses from graph vio states.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a graph_vio_state.
def adjusted_graph_vio_poses_from_graph_vio_states(
    graph_vio_states, groundtruth_poses, max_diff=0.1
):
    graph_vio_state_times = np.array([state.timestamp for state in graph_vio_states])
    world_T_vio = None
    for groundtruth_pose in groundtruth_poses:
        closest_matching_graph_vio_state_index = np.argmin(
            np.abs(graph_vio_state_times - groundtruth_pose.timestamp)
        )
        timestamp_diff = np.abs(
            graph_vio_state_times[closest_matching_graph_vio_state_index]
            - groundtruth_pose.timestamp
        )
        if timestamp_diff <= max_diff:
            closest_graph_vio_state = graph_vio_states[
                closest_matching_graph_vio_state_index
            ]
            world_T_vio = (
                groundtruth_pose
                * graph_vio_states[
                    closest_matching_graph_vio_state_index
                ].pose_with_covariance.inverse()
            )
            break
    if not world_T_vio:
        print("Failed to find corresponding groundtruth pose to graph VIO poses")
        sys.exit(0)
    adjusted_graph_vio_poses = []
    for graph_vio_state in graph_vio_states:
        adjusted_pose = world_T_vio * states.TimestampedPose(
            graph_vio_state.pose_with_covariance.orientation,
            graph_vio_state.pose_with_covariance.position,
            graph_vio_state.timestamp,
        )
        adjusted_graph_vio_poses.append(
            states.TimestampedPose(
                adjusted_pose.orientation,
                adjusted_pose.position,
                graph_vio_state.timestamp,
            )
        )
    return adjusted_graph_vio_poses


# Return list of imu bias extrapolated poses.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a imu bias extrapolated pose.
def absolute_poses_from_imu_bias_extrapolated_poses(
    imu_bias_extrapolated_poses, groundtruth_poses, max_diff=0.1
):
    imu_bias_times = np.array(
        [imu_bias.timestamp for imu_bias in imu_bias_extrapolated_poses]
    )
    world_T_vio = None
    for groundtruth_pose in groundtruth_poses:
        closest_matching_imu_bias_index = np.argmin(
            np.abs(imu_bias_times - groundtruth_pose.timestamp)
        )
        timestamp_diff = np.abs(
            imu_bias_times[closest_matching_imu_bias_index] - groundtruth_pose.timestamp
        )
        if timestamp_diff <= max_diff:
            closest_imu_bias = imu_bias_extrapolated_poses[
                closest_matching_imu_bias_index
            ]
            world_T_vio = (
                groundtruth_pose
                * imu_bias_extrapolated_poses[closest_matching_imu_bias_index].inverse()
            )
            break
    if not world_T_vio:
        print("Failed to find corresponding groundtruth pose to graph VIO poses")
        sys.exit(0)
    adjusted_imu_bias_poses = []
    for imu_bias in imu_bias_extrapolated_poses:
        adjusted_pose = world_T_vio * states.TimestampedPose(
            imu_bias.orientation, imu_bias.position, imu_bias.timestamp
        )
        adjusted_imu_bias_poses.append(
            states.TimestampedPose(
                adjusted_pose.orientation, adjusted_pose.position, imu_bias.timestamp
            )
        )
    return adjusted_imu_bias_poses


# Return list of absolute poses.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a imu bias extrapolated pose.
def absolute_poses_from_relative_poses(relative_poses, groundtruth_poses, max_diff=0.1):
    relative_pose_times = np.array(
        [relative_pose.timestamp for relative_pose in relative_poses]
    )
    world_T_odom = None
    for groundtruth_pose in groundtruth_poses:
        closest_matching_relative_pose_index = np.argmin(
            np.abs(relative_pose_times - groundtruth_pose.timestamp)
        )
        timestamp_diff = np.abs(
            relative_pose_times[closest_matching_relative_pose_index]
            - groundtruth_pose.timestamp
        )
        if timestamp_diff <= max_diff:
            closest_relative_pose = relative_poses[closest_matching_relative_pose_index]
            world_T_odom = (
                groundtruth_pose
                * relative_poses[closest_matching_relative_pose_index].inverse()
            )
            break
    if not world_T_odom:
        print("Failed to find corresponding groundtruth pose to graph VIO poses")
        sys.exit(0)
    adjusted_relative_pose_poses = []
    previous_relative_pose = None
    for relative_pose in relative_poses:
        pose = states.TimestampedPose(
            relative_pose.orientation, relative_pose.position, relative_pose.timestamp
        )
        if previous_relative_pose:
            pose = previous_relative_pose * pose
        adjusted_pose = world_T_odom * pose
        adjusted_relative_pose_poses.append(
            states.TimestampedPose(
                adjusted_pose.orientation,
                adjusted_pose.position,
                relative_pose.timestamp,
            )
        )
        previous_relative_pose = pose
    return adjusted_relative_pose_poses


# Return list of poses from integrated velocities.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a velocity.
def absolute_poses_from_integrated_velocities(
    timestamped_velocities,
    groundtruth_poses,
    frame_change_velocity=False,
    max_diff=0.1,
    poses=None,
):
    times = np.array([velocity.timestamp for velocity in timestamped_velocities])
    start_index = None
    start_groundtruth_index = None
    starting_groundtruth_pose = None
    for i in range(len(groundtruth_poses)):
        closest_matching_index = np.argmin(
            np.abs(times - groundtruth_poses[i].timestamp)
        )
        timestamp_diff = np.abs(
            times[closest_matching_index] - groundtruth_poses[i].timestamp
        )
        if timestamp_diff <= max_diff:
            start_index = closest_matching_index
            starting_groundtruth_pose = groundtruth_poses[i]
            start_groundtruth_index = i
            break
    if start_index is None:
        print("Failed to find corresponding groundtruth pose to graph VIO poses")
        sys.exit(0)
    if frame_change_velocity:
        return integrate_velocities_with_frame_change(
            timestamped_velocities[start_index:],
            starting_groundtruth_pose,
            groundtruth_poses,
            start_groundtruth_index,
            poses[start_index:],
        )
    else:
        return integrate_velocities(
            timestamped_velocities[start_index:], starting_groundtruth_pose
        )


# Return list of poses from integrated graph_vio velocities.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a velocity.
def absolute_poses_from_integrated_graph_vio_state_velocities(
    graph_vio_states, groundtruth_poses, max_diff=0.1
):
    velocities = [
        states.TimestampedVelocity(
            graph_vio_state.velocity_with_covariance.x,
            graph_vio_state.velocity_with_covariance.y,
            graph_vio_state.velocity_with_covariance.z,
            graph_vio_state.timestamp,
        )
        for graph_vio_state in graph_vio_states
    ]
    poses = [
        states.TimestampedPose(
            graph_vio_state.pose_with_covariance.orientation,
            graph_vio_state.pose_with_covariance.position,
            graph_vio_state.timestamp,
        )
        for graph_vio_state in graph_vio_states
    ]

    return absolute_poses_from_integrated_velocities(
        velocities, groundtruth_poses, True, max_diff, poses
    )


# Return list of poses from integrated extrapolated loc velocities.
# Poses are adjusted to start at the corresponding groundtruth pose at the earliest corresponding timestamp
# so they can be plotted against groundtruth poses. The earliest corresponding timestamp is the first timestamp
# with a dt <= max_diff compared to a velocity.
def absolute_poses_from_integrated_extrapolated_loc_state_velocities(
    extrapolated_loc_states, groundtruth_poses, max_diff=0.1
):
    velocities = [
        states.TimestampedVelocity(
            extrapolated_loc_state.velocity.x,
            extrapolated_loc_state.velocity.y,
            extrapolated_loc_state.velocity.z,
            extrapolated_loc_state.timestamp,
        )
        for extrapolated_loc_state in extrapolated_loc_states
    ]
    return absolute_poses_from_integrated_velocities(
        velocities, groundtruth_poses, False, max_diff
    )


# Integrates graph vio velocities and appends these to a starting pose to generate absolute pose estimates
def integrate_velocities(velocities, starting_pose):
    # Calculate relative x,y,z increments using v*dt for each increment
    # Succesively add these increments to starting pose to generate future poses
    x = starting_pose.position.x
    y = starting_pose.position.y
    z = starting_pose.position.z
    integrated_poses = []
    for i in range(len(velocities) - 1):
        integrated_poses.append(
            states.TimestampedPose(
                scipy.spatial.transform.Rotation.from_rotvec([0, 0, 0]),
                states.Position([x, y, z]),
                velocities[i].timestamp,
            )
        )
        dt = velocities[i + 1].timestamp - velocities[i].timestamp
        dx = dt * velocities[i].x
        dy = dt * velocities[i].y
        dz = dt * velocities[i].z
        x += dx
        y += dy
        z += dz
    return integrated_poses


# Integrates graph vio velocities and appends these to a starting pose to generate absolute pose estimates
# Assumes velocities are in a separate odom frame and uses the closest groundtruth pose to rotete
# each velocity to the world frame.
def integrate_velocities_with_frame_change(
    velocities,
    starting_pose,
    groundtruth_poses,
    starting_groundtruth_index,
    poses,
    max_diff=0.5,
):
    # Calculate relative x,y,z increments using v*dt for each increment
    # Succesively add these increments to starting pose to generate future poses
    x = starting_pose.position.x
    y = starting_pose.position.y
    z = starting_pose.position.z
    groundtruth_index = starting_groundtruth_index
    # Initialize world_R_odom
    world_R_odom = groundtruth_poses[groundtruth_index] * poses[0].inverse()
    world_R_odom.position = states.Position([0, 0, 0])
    integrated_poses = []
    for i in range(len(velocities) - 1):
        if groundtruth_index + 1 < len(groundtruth_poses):
            # Update groundtruth index if successive groundtruth pose timestamp is closer to
            # current velocity estimate
            current_gt_time_diff = abs(
                groundtruth_poses[groundtruth_index].timestamp - velocities[i].timestamp
            )
            next_gt_time_diff = abs(
                groundtruth_poses[groundtruth_index + 1].timestamp
                - velocities[i].timestamp
            )
            if next_gt_time_diff < current_gt_time_diff:
                groundtruth_index = groundtruth_index + 1
                # Update gt world_R_odom if time diffs between gt pose and velocity are close enough
                if next_gt_time_diff < max_diff:
                    # Update world_R_odom
                    world_R_odom = (
                        groundtruth_poses[groundtruth_index] * poses[i].inverse()
                    )
                    world_R_odom.position = states.Position([0, 0, 0])
        integrated_poses.append(
            states.TimestampedPose(
                scipy.spatial.transform.Rotation.from_rotvec([0, 0, 0]),
                states.Position([x, y, z]),
                velocities[i].timestamp,
            )
        )
        dt = velocities[i + 1].timestamp - velocities[i].timestamp
        dx = dt * velocities[i].x
        dy = dt * velocities[i].y
        dz = dt * velocities[i].z
        # Frame change velocity
        odom_F_rel_velocity_pose = states.TimestampedPose(
            scipy.spatial.transform.Rotation.from_rotvec([0, 0, 0]),
            states.Position([dx, dy, dz]),
            velocities[i].timestamp,
        )
        world_F_rel_velocity_pose = world_R_odom * odom_F_rel_velocity_pose
        dx = world_F_rel_velocity_pose.position.x
        dy = world_F_rel_velocity_pose.position.y
        dz = world_F_rel_velocity_pose.position.z

        x += dx
        y += dy
        z += dz
    return integrated_poses


# Return list of ml measurement counts from graph loc states
def ml_feature_counts_from_graph_loc_states(graph_loc_states):
    return [
        graph_loc_state.num_detected_ml_features for graph_loc_state in graph_loc_states
    ]


# Return list of ar measurement counts from graph loc states
def ar_feature_counts_from_graph_loc_states(graph_loc_states):
    return [
        graph_loc_state.num_detected_ar_features for graph_loc_state in graph_loc_states
    ]


# Return list of optical flow feature counts from graph vio states
def optical_flow_feature_counts_from_graph_vio_states(graph_vio_states):
    return [
        graph_vio_state.num_detected_of_features for graph_vio_state in graph_vio_states
    ]


# Return list of optical flow factor counts from graph vio states
def optical_flow_factor_counts_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.num_of_factors for graph_vio_state in graph_vio_states]


# Return list of depth factor counts from graph vio states
def depth_factor_counts_from_graph_vio_states(graph_vio_states):
    return [graph_vio_state.num_depth_factors for graph_vio_state in graph_vio_states]


# Return list of ml pose factor counts from graph loc states
def ml_pose_factor_counts_from_graph_loc_states(graph_loc_states):
    return [graph_loc_state.num_ml_pose_factors for graph_loc_state in graph_loc_states]


# Return list of ml projection factor counts from graph loc states
def ml_projection_factor_counts_from_graph_loc_states(graph_loc_states):
    return [
        graph_loc_state.num_ml_projection_factors
        for graph_loc_state in graph_loc_states
    ]


# Return list of standstill occurances times from states
def standstill_from_states(states):
    return [int(state.standstill) for state in states]


# Return list of optimization iterations from states
def optimization_iterations_from_states(states):
    return [state.optimization_iterations for state in states]


# Return list of num states from states
def num_states_from_states(states):
    return [state.num_states for state in states]


# Return list of duration times from states
def durations_from_states(states):
    return [state.duration for state in states]


# Return list of optimization times from states
def optimization_times_from_states(states):
    return [state.optimization_time for state in states]


# Return list of update times from states
def update_times_from_states(states):
    return [state.update_time for state in states]


# Return list of num features from depth odometries
def num_features_from_depth_odometries(depth_odometries):
    return [depth_odometry.num_features for depth_odometry in depth_odometries]


# Return list of runtimes from depth odometries
def runtimes_from_depth_odometries(depth_odometries):
    return [depth_odometry.runtime for depth_odometry in depth_odometries]


# Return list of timestamped poses from depth odometries
def poses_from_depth_odometries(depth_odometries):
    return [
        states.TimestampedPose(
            depth_odometry.pose_with_covariance.orientation,
            depth_odometry.pose_with_covariance.position,
            depth_odometry.timestamp,
        )
        for depth_odometry in depth_odometries
    ]


# Return list of timestamped poses from graph loc states
def poses_from_graph_loc_states(graph_loc_states):
    return [
        states.TimestampedPose(
            graph_loc_state.pose_with_covariance.orientation,
            graph_loc_state.pose_with_covariance.position,
            graph_loc_state.timestamp,
        )
        for graph_loc_state in graph_loc_states
    ]


# Return list of timestamped poses from extrapolated loc states
def poses_from_extrapolated_loc_states(extrapolated_loc_states):
    return [
        states.TimestampedPose(
            extrapolated_loc_state.pose.orientation,
            extrapolated_loc_state.pose.position,
            extrapolated_loc_state.timestamp,
        )
        for extrapolated_loc_state in extrapolated_loc_states
    ]


def velocity_plotter_from_graph_vio_states(graph_vio_states):
    xs, ys, zs = xyz_velocity_vectors_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states)
    return plotters.Vector3dPlotter(
        "Graph VIO Velocity", times, xs, ys, zs, ["X", "Y", "Z"], marker="o"
    )


def velocity_plotter_from_extrapolated_loc_states(extrapolated_loc_states):
    xs, ys, zs = xyz_velocity_vectors_from_extrapolated_loc_states(
        extrapolated_loc_states
    )
    times = times_from_timestamped_objects(extrapolated_loc_states)
    return plotters.Vector3dPlotter(
        "Extrapolated Loc Velocity", times, xs, ys, zs, ["X", "Y", "Z"]
    )


def acceleration_plotter_from_extrapolated_loc_states(extrapolated_loc_states):
    xs, ys, zs = xyz_acceleration_vectors_from_extrapolated_loc_states(
        extrapolated_loc_states
    )
    times = times_from_timestamped_objects(extrapolated_loc_states)
    return plotters.Vector3dPlotter(
        "Extrapolated Loc Acceleration", times, xs, ys, zs, ["X", "Y", "Z"]
    )


def acceleration_plotter_from_imu_accelerations(imu_accelerations):
    xs, ys, zs = xyz_acceleration_vectors_from_imu_accelerations(imu_accelerations)
    times = times_from_timestamped_objects(imu_accelerations)
    return plotters.Vector3dPlotter(
        "Raw IMU Acceleration", times, xs, ys, zs, ["X", "Y", "Z"]
    )


def accel_bias_plotter_from_graph_vio_states(graph_vio_states):
    xs, ys, zs = xyz_accel_bias_vectors_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states)
    return plotters.Vector3dPlotter(
        "Graph VIO Accel. Bias", times, xs, ys, zs, ["X", "Y", "Z"]
    )


def gyro_bias_plotter_from_graph_vio_states(graph_vio_states):
    xs, ys, zs = xyz_gyro_bias_vectors_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states)
    return plotters.Vector3dPlotter(
        "Graph VIO Gyro Bias", times, xs, ys, zs, ["X", "Y", "Z"]
    )


def ml_feature_count_plotter_from_graph_loc_states(graph_loc_states):
    counts = ml_feature_counts_from_graph_loc_states(graph_loc_states)
    times = times_from_timestamped_objects(graph_loc_states)
    return plotters.ValuePlotter(
        "Graph Loc ML Feature Counts", times, counts, "Time (s)", "Num Features", "ML"
    )


def ar_feature_count_plotter_from_graph_loc_states(graph_loc_states):
    counts = ar_feature_counts_from_graph_loc_states(graph_loc_states)
    times = times_from_timestamped_objects(graph_loc_states)
    return plotters.ValuePlotter(
        "Graph Loc AR Feature Counts", times, counts, "Time (s)", "Num Features", "AR"
    )


def optical_flow_feature_count_plotter_from_graph_vio_states(graph_vio_states):
    counts = optical_flow_feature_counts_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states)
    return plotters.ValuePlotter(
        "Graph VIO OF Counts", times, counts, "Time (s)", "Num Features", "OF"
    )


def optical_flow_factor_count_plotter_from_graph_vio_states(graph_vio_states):
    counts = optical_flow_factor_counts_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states)
    return plotters.ValuePlotter(
        "Graph VIO OF Factors", times, counts, "Time (s)", "Num Factors", "OF"
    )


def depth_factor_count_plotter_from_graph_vio_states(graph_vio_states):
    counts = depth_factor_counts_from_graph_vio_states(graph_vio_states)
    times = times_from_timestamped_objects(graph_vio_states)
    return plotters.ValuePlotter(
        "Graph VIO Depth Factors", times, counts, "Time (s)", "Num Factors", "Depth"
    )


def ml_pose_factor_count_plotter_from_graph_loc_states(graph_loc_states):
    counts = ml_pose_factor_counts_from_graph_loc_states(graph_loc_states)
    times = times_from_timestamped_objects(graph_loc_states)
    return plotters.ValuePlotter(
        "Graph Loc ML Pose Factors", times, counts, "Time (s)", "Num Factors", "ML"
    )


def ml_projection_factor_count_plotter_from_graph_loc_states(graph_loc_states):
    counts = ml_projection_factor_counts_from_graph_loc_states(graph_loc_states)
    times = times_from_timestamped_objects(graph_loc_states)
    return plotters.ValuePlotter(
        "Graph Loc ML Projection Factors",
        times,
        counts,
        "Time (s)",
        "Num Factors",
        "ML",
    )


def standstill_plotter_from_states(states):
    standstill = standstill_from_states(states)
    times = times_from_timestamped_objects(states)
    return plotters.ValuePlotter(
        "Standstill Occurances",
        times,
        standstill,
        "Time (s)",
        "Standstill (True/False)",
        "Standstill",
    )


def optimization_iterations_plotter_from_states(states):
    optimization_iterations = optimization_iterations_from_states(states)
    times = times_from_timestamped_objects(states)
    return plotters.ValuePlotter(
        "Num Graph Optimization Iterations",
        times,
        optimization_iterations,
        "Time (s)",
        "Num Iterations",
        "Num Optimization Iterations",
    )


def num_states_plotter_from_states(states):
    num_states = num_states_from_states(states)
    times = times_from_timestamped_objects(states)
    return plotters.ValuePlotter(
        "Num Graph States", times, num_states, "Time (s)", "Num States", "Num States"
    )


def duration_plotter_from_states(states):
    durations = durations_from_states(states)
    times = times_from_timestamped_objects(states)
    return plotters.ValuePlotter(
        "Duration", times, durations, "Time (s)", "Duration (s)", "Duration"
    )


def optimization_time_plotter_from_states(states):
    optimization_times = optimization_times_from_states(states)
    times = times_from_timestamped_objects(states)
    return plotters.ValuePlotter(
        "Optimization Times",
        times,
        optimization_times,
        "Time (s)",
        "Opt. Time (s)",
        "Opt. Time",
    )


def update_time_plotter_from_states(states):
    update_times = update_times_from_states(states)
    times = times_from_timestamped_objects(states)
    return plotters.ValuePlotter(
        "Update Times",
        times,
        update_times,
        "Time (s)",
        "Update Time (s)",
        "Update Time",
    )


def runtime_plotter_from_depth_odometries(depth_odometries):
    runtimes = runtimes_from_depth_odometries(depth_odometries)
    times = times_from_timestamped_objects(depth_odometries)
    return plotters.ValuePlotter(
        "Depth Odometry Runtime",
        times,
        runtimes,
        "Time (s)",
        "Runtimes (s)",
        "Depth Odometry Runtimes",
    )


def num_features_plotter_from_depth_odometries(depth_odometries):
    num_features = num_features_from_depth_odometries(depth_odometries)
    times = times_from_timestamped_objects(depth_odometries)
    return plotters.ValuePlotter(
        "Depth Odometry Num Features",
        times,
        num_features,
        "Time (s)",
        "Num Features",
        "Depth Odometry Num Features",
    )
