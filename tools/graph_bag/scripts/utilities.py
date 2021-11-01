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

import datetime
import glob
import os
import subprocess

import numpy as np
import pandas as pd
import poses
import rosbag


# Forward errors so we can recover failures
# even when running commands through multiprocessing
# pooling
def full_traceback(func):
    import functools
    import traceback

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            msg = "{}\n\nOriginal {}".format(e, traceback.format_exc())
            raise type(e)(msg)

    return wrapper


def get_files(directory, file_string):
    return glob.glob(os.path.join(directory, file_string))


def get_files_recursive(directory, file_string):
    subdirectory_csv_files = []
    _, subdirectories, _ = next(os.walk(directory))
    for subdirectory in subdirectories:
        subdirectory_path = os.path.join(directory, subdirectory)
        for subdirectory_csv_file in get_files(subdirectory_path, file_string):
            subdirectory_csv_files.append(subdirectory_csv_file)
    return subdirectory_csv_files


def create_directory(directory=None):
    if directory == None:
        directory = os.path.join(
            os.getcwd(), datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        )
    if os.path.exists(directory):
        print((directory + " already exists!"))
        exit()
    os.makedirs(directory)
    return directory


def load_dataframe(files):
    dataframes = [pd.read_csv(file) for file in files]
    dataframe = pd.concat(dataframes)
    return dataframe

def run_command_and_save_output(command, output_filename):
      with open(output_filename, 'w') as output_file:
              subprocess.call(command, shell=True, stdout=output_file, stderr=output_file)

# TODO(rsoussan): Move these to different utilities file
def get_topic_rates(
    bag_name,
    topic,
    min_time_diff_for_gap,
    use_header_time=True,
    verbose=False,
    ignore_zero_time_diffs=True,
):
    with rosbag.Bag(bag_name, "r") as bag:
        last_time = 0.0
        gaps = 0
        time_diffs = []
        for _, msg, t in bag.read_messages([topic]):
            time = (
                msg.header.stamp.secs + msg.header.stamp.nsecs * 1.0e-9
                if use_header_time
                else t.secs + t.nsecs * 1.0e-9
            )
            time_diff = time - last_time
            if last_time != 0 and time_diff >= min_time_diff_for_gap:
                if verbose:
                    print(
                        (
                            topic
                            + " Gap: time: "
                            + str(time)
                            + ", last_time: "
                            + str(last_time)
                            + ", diff: "
                            + str(time_diff)
                        )
                    )
                gaps += 1
            if last_time != 0 and (time_diff != 0 or not ignore_zero_time_diffs):
                time_diffs.append(time_diff)
            last_time = time

        mean_time_diff = np.mean(time_diffs)
        min_time_diff = np.min(time_diffs)
        max_time_diff = np.max(time_diffs)
        stddev_time_diff = np.std(time_diffs)
        if verbose:
            if use_header_time:
                print("Using Header time.")
            else:
                print("Using Receive time.")
            print(
                (
                    "Found "
                    + str(gaps)
                    + " time diffs >= "
                    + str(min_time_diff_for_gap)
                    + " secs."
                )
            )
            print(("Mean time diff: " + str(mean_time_diff)))
            print(("Min time diff: " + str(min_time_diff)))
            print(("Max time diff: " + str(max_time_diff)))
            print(("Stddev time diff: " + str(stddev_time_diff)))


def integrate_velocities(localization_states):
    delta_times = [
        j - i
        for i, j in zip(localization_states.times[:-1], localization_states.times[1:])
    ]
    # Make sure times are same length as velocities, ignore last velocity
    delta_times.append(0)
    integrated_positions = poses.Poses("Integrated Graph Velocities", "")
    # TODO(rsoussan): Integrate angular velocities?
    # TODO(rsoussan): central difference instead?
    x_increments = [
        velocity * delta_t
        for velocity, delta_t in zip(localization_states.velocities.xs, delta_times)
    ]
    cumulative_x_increments = np.cumsum(x_increments)
    integrated_positions.positions.xs = [
        localization_states.positions.xs[0] + cumulative_x_increment
        for cumulative_x_increment in cumulative_x_increments
    ]
    y_increments = [
        velocity * delta_t
        for velocity, delta_t in zip(localization_states.velocities.ys, delta_times)
    ]
    cumulative_y_increments = np.cumsum(y_increments)
    integrated_positions.positions.ys = [
        localization_states.positions.ys[0] + cumulative_y_increment
        for cumulative_y_increment in cumulative_y_increments
    ]
    z_increments = [
        velocity * delta_t
        for velocity, delta_t in zip(localization_states.velocities.zs, delta_times)
    ]
    cumulative_z_increments = np.cumsum(z_increments)
    integrated_positions.positions.zs = [
        localization_states.positions.zs[0] + cumulative_z_increment
        for cumulative_z_increment in cumulative_z_increments
    ]

    # Add start positions
    integrated_positions.positions.xs.insert(0, localization_states.positions.xs[0])
    integrated_positions.positions.ys.insert(0, localization_states.positions.ys[0])
    integrated_positions.positions.zs.insert(0, localization_states.positions.zs[0])

    # Remove last elements (no timestamp for these)
    del integrated_positions.positions.xs[-1]
    del integrated_positions.positions.ys[-1]
    del integrated_positions.positions.zs[-1]

    integrated_positions.times = localization_states.times
    return integrated_positions
