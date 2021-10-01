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

import bisect
import math

import numpy as np
import poses
import scipy.spatial.transform


# Assumes poses_a and poses_b are sorted in time
def get_same_timestamp_poses(
    poses_a,
    poses_b,
    add_orientations=True,
    abs_tol=0,
    rel_start_time=0,
    rel_end_time=-1,
):
    trimmed_poses_a = poses.Poses(poses_a.pose_type, poses_a.topic)
    trimmed_poses_b = poses.Poses(poses_b.pose_type, poses_b.topic)
    poses_a_size = len(poses_a.times)
    poses_b_size = len(poses_b.times)
    a_index = 0
    b_index = 0
    # Increment a and b index as needed.  Add same timestamped poses to trimmed poses containers
    while (a_index < poses_a_size) and (b_index < poses_b_size):
        a_time = poses_a.times[a_index]
        b_time = poses_b.times[b_index]

        # Check if times are within given start and end time bounds
        if a_time < rel_start_time:
            a_index += 1
            continue
        if b_time < rel_start_time:
            b_index += 1
            continue
        # rel_end_time less than zero indicates no bound on end time
        if rel_end_time >= 0:
            if a_time > rel_end_time or b_time > rel_end_time:
                break

        if np.isclose(a_time, b_time, rtol=0, atol=abs_tol):
            trimmed_poses_a.positions.add_vector3d(
                poses_a.positions.get_vector3d(a_index)
            )
            trimmed_poses_a.times.append(poses_a.times[a_index])
            trimmed_poses_b.positions.add_vector3d(
                poses_b.positions.get_vector3d(b_index)
            )
            trimmed_poses_b.times.append(poses_b.times[b_index])
            if add_orientations:
                trimmed_poses_a.orientations.add_euler(
                    poses_a.orientations.get_euler(a_index)
                )
                trimmed_poses_b.orientations.add_euler(
                    poses_b.orientations.get_euler(b_index)
                )
            a_index += 1
            b_index += 1
        elif a_time < b_time:
            a_index += 1
        else:
            b_index += 1
    return trimmed_poses_a, trimmed_poses_b


def position_squared_difference(a, b):
    difference_vec = np.subtract(a, b)
    return np.inner(difference_vec, difference_vec)


# Uses square of radian difference of rotations in axis angle representation
def orientation_squared_difference(world_R_a, world_R_b):
    a_R_b = world_R_a.inv() * world_R_b
    return np.inner(a_R_b.as_rotvec(), a_R_b.as_rotvec())


# RMSE between two sequences of poses. Only uses poses with the same timestamp
def rmse_timestamped_poses(
    poses_a,
    poses_b,
    add_orientation_rmse=True,
    abs_tol=0,
    rel_start_time=0,
    rel_end_time=-1,
):
    trimmed_poses_a, trimmed_poses_b = get_same_timestamp_poses(
        poses_a, poses_b, add_orientation_rmse, abs_tol, rel_start_time, rel_end_time
    )
    assert len(trimmed_poses_a.times) == len(
        trimmed_poses_b.times
    ), "Length mismatch of poses"
    num_poses = len(trimmed_poses_a.times)
    mean_squared_position_error = 0
    mean_squared_orientation_error = 0
    for index in range(num_poses):
        # Position Error
        a_vec = trimmed_poses_a.positions.get_numpy_vector(index)
        b_vec = trimmed_poses_b.positions.get_numpy_vector(index)
        position_squared_error = position_squared_difference(a_vec, b_vec)
        # Use rolling mean to avoid overflow
        mean_squared_position_error += (
            position_squared_error - mean_squared_position_error
        ) / (index + 1)
        # Orientation Error
        if add_orientation_rmse:
            a_rot = trimmed_poses_a.orientations.get_rotation(index)
            b_rot = trimmed_poses_b.orientations.get_rotation(index)
            orientation_squared_error = orientation_squared_difference(a_rot, b_rot)
            mean_squared_orientation_error += (
                orientation_squared_error - mean_squared_orientation_error
            ) / (index + 1)
    position_rmse = math.sqrt(mean_squared_position_error)
    orientation_rmse = math.sqrt(mean_squared_orientation_error)
    return position_rmse, orientation_rmse


# Relative RMSE between two sequences of poses. Only uses poses with the same timestamp
def rmse_timestamped_poses_relative(
    poses_a,
    poses_b,
    add_orientation_rmse=True,
    abs_tol=0,
    rel_start_time=0,
    rel_end_time=-1,
    min_relative_elapsed_time=10,
    max_relative_elapsed_time=20,
):
    trimmed_poses_a, trimmed_poses_b = get_same_timestamp_poses(
        poses_a, poses_b, add_orientation_rmse, abs_tol, rel_start_time, rel_end_time
    )
    assert len(trimmed_poses_a.times) == len(
        trimmed_poses_b.times
    ), "Length mismatch of poses"
    num_poses = len(trimmed_poses_a.times)
    mean_squared_position_error = 0
    mean_squared_orientation_error = 0
    count = 0
    for index1 in range(num_poses):
        # Position Error
        a_vec1 = trimmed_poses_a.positions.get_numpy_vector(index1)
        b_vec1 = trimmed_poses_b.positions.get_numpy_vector(index1)
        time1 = trimmed_poses_a.times[index1]
        index2 = bisect.bisect_left(
            trimmed_poses_a.times, time1 + min_relative_elapsed_time
        )
        if index2 == len(trimmed_poses_a.times):
            continue
        time2 = trimmed_poses_a.times[index2]
        if time2 - time1 > max_relative_elapsed_time:
            continue
        a_vec2 = trimmed_poses_a.positions.get_numpy_vector(index2)
        b_vec2 = trimmed_poses_b.positions.get_numpy_vector(index2)
        a_rel_vec = a_vec2 - a_vec1
        b_rel_vec = b_vec2 - b_vec1

        position_squared_error = position_squared_difference(a_rel_vec, b_rel_vec)
        count += 1
        # Use rolling mean to avoid overflow
        mean_squared_position_error += (
            position_squared_error - mean_squared_position_error
        ) / float(count)
        # Orientation Error
        if add_orientation_rmse:
            # TODO(rsoussan): Add relative calculations for orientations
            a_rot = trimmed_poses_a.orientations.get_rotation(index1)
            b_rot = trimmed_poses_b.orientations.get_rotation(index1)
            orientation_squared_error = orientation_squared_difference(a_rot, b_rot)
            mean_squared_orientation_error += (
                orientation_squared_error - mean_squared_orientation_error
            ) / float(count)
    position_rmse = math.sqrt(mean_squared_position_error)
    orientation_rmse = math.sqrt(mean_squared_orientation_error)
    return position_rmse, orientation_rmse
