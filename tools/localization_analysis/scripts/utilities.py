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

import pose
import poses


def make_absolute_poses_from_relative_poses(absolute_poses, relative_poses, name):
    starting_relative_time = relative_poses.times[0]
    np_times = np.array(absolute_poses.times)
    closest_index = np.argmin(np.abs(np_times - starting_relative_time))
    start_pose = absolute_poses.pose(closest_index)
    new_pose = start_pose
    new_poses_list = [start_pose]
    new_poses_times = [absolute_poses.times[closest_index]]
    for index in range(len(relative_poses.times)):
        relative_pose = relative_poses.pose(index)
        new_pose = new_pose * relative_pose
        new_poses_list.append(new_pose)
        new_poses_times.append(relative_poses.times[index])
    new_poses = poses.Poses(name, "")
    new_poses.init_from_poses(new_poses_list, new_poses_times)
    return new_poses


def integrate_velocities(localization_states):
    delta_times = [
        j - i
        for i, j in zip(localization_states.times[:-1], localization_states.times[1:])
    ]
    # Make sure times are same length as velocities, ignore last velocity
    delta_times.append(0)
    # TODO(rsoussan): Integrate angular velocities?
    # TODO(rsoussan): central difference instead?
    x_increments = [
        velocity * delta_t
        for velocity, delta_t in zip(localization_states.velocities.xs, delta_times)
    ]
    y_increments = [
        velocity * delta_t
        for velocity, delta_t in zip(localization_states.velocities.ys, delta_times)
    ]
    z_increments = [
        velocity * delta_t
        for velocity, delta_t in zip(localization_states.velocities.zs, delta_times)
    ]

    return add_increments_to_absolute_pose(
        x_increments,
        y_increments,
        z_increments,
        localization_states.positions.xs[0],
        localization_states.positions.ys[0],
        localization_states.positions.zs[0],
        localization_states.times,
        "Integrated Graph Velocities",
    )


def add_increments_to_absolute_pose(
    x_increments,
    y_increments,
    z_increments,
    starting_x,
    starting_y,
    starting_z,
    times,
    poses_name="Increment Poses",
):
    integrated_positions = poses.Poses(poses_name, "")
    cumulative_x_increments = np.cumsum(x_increments)
    integrated_positions.positions.xs = [
        starting_x + cumulative_x_increment
        for cumulative_x_increment in cumulative_x_increments
    ]
    cumulative_y_increments = np.cumsum(y_increments)
    integrated_positions.positions.ys = [
        starting_y + cumulative_y_increment
        for cumulative_y_increment in cumulative_y_increments
    ]
    cumulative_z_increments = np.cumsum(z_increments)
    integrated_positions.positions.zs = [
        starting_z + cumulative_z_increment
        for cumulative_z_increment in cumulative_z_increments
    ]

    # Add start positions
    integrated_positions.positions.xs.insert(0, starting_x)
    integrated_positions.positions.ys.insert(0, starting_y)
    integrated_positions.positions.zs.insert(0, starting_z)

    # Remove last elements (no timestamp for these)
    del integrated_positions.positions.xs[-1]
    del integrated_positions.positions.ys[-1]
    del integrated_positions.positions.zs[-1]

    integrated_positions.times = times
    return integrated_positions
