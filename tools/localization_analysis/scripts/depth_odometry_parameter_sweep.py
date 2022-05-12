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
"""
Runs a parameter sweep for depth odometry using the provided bagfile.
Parameters to sweep on are set in the make_value_ranges function and applied to 
the localization/depth_odometry.config file.
All combinations of provided parameters are used for the sweep and the results
are plotted for various RMSEs. 
"""


import argparse
import csv
import itertools
import math
import multiprocessing
import os

import config_creator
import localization_common.utilities as lu
import numpy as np
import parameter_sweep_utilities
import plot_parameter_sweep_results


# Run depth odometry with values.
# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@lu.full_traceback
def test_values(
    values,
    job_id,
    value_names,
    output_dir,
    bag_file,
    config_path,
    robot_config,
    world,
    groundtruth_bagfile,
    rmse_rel_start_time,
    rmse_rel_end_time,
):
    new_output_dir = os.path.join(output_dir, str(job_id))
    os.mkdir(new_output_dir)
    depth_odometry_config_filepath = os.path.join(
        config_path, "config", "localization/depth_odometry.config"
    )
    new_depth_odometry_config_filepath = os.path.join(
        new_output_dir, "depth_odometry.config"
    )
    config_creator.make_config(
        values,
        value_names,
        depth_odometry_config_filepath,
        new_depth_odometry_config_filepath,
    )
    output_bag = os.path.join(new_output_dir, "results.bag")
    output_stats_file = os.path.join(new_output_dir, "depth_odom_stats.csv")
    depth_odometry_config_prefix = new_output_dir + "/"
    run_command = (
        "rosrun localization_analysis run_depth_odometry_adder "
        + bag_file
        + " "
        + config_path
        + " -r "
        + robot_config
        + " -o "
        + output_bag
        + " -w "
        + world
        + " -p "
        + depth_odometry_config_prefix
    )
    os.system(run_command)
    output_pdf_file = os.path.join(new_output_dir, str(job_id) + "_output.pdf")
    output_csv_file = os.path.join(new_output_dir, "depth_odom_stats.csv")
    plot_command = (
        "rosrun localization_analysis plot_results.py "
        + output_bag
        + " --output-file "
        + output_pdf_file
        + " --output-csv-file "
        + output_csv_file
        + " -g "
        + groundtruth_bagfile
        + " --rmse-rel-start-time "
        + str(rmse_rel_start_time)
        + " --rmse-rel-end-time "
        + str(rmse_rel_end_time)
    )
    os.system(plot_command)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def test_values_helper(zipped_vals):
    return test_values(*zipped_vals)


def parameter_sweep(
    all_value_combos,
    value_names,
    output_dir,
    bag_file,
    config_path,
    robot_config,
    world,
    groundtruth_bagfile,
    rmse_rel_start_time=0,
    rmse_rel_end_time=-1,
):
    job_ids = list(range(len(all_value_combos)))
    num_processes = 15
    pool = multiprocessing.Pool(num_processes)
    # izip arguments so we can pass as one argument to pool worker
    pool.map(
        test_values_helper,
        list(
            zip(
                all_value_combos,
                job_ids,
                itertools.repeat(value_names),
                itertools.repeat(output_dir),
                itertools.repeat(bag_file),
                itertools.repeat(config_path),
                itertools.repeat(robot_config),
                itertools.repeat(world),
                itertools.repeat(groundtruth_bagfile),
                itertools.repeat(rmse_rel_start_time),
                itertools.repeat(rmse_rel_end_time),
            )
        ),
    )
    parameter_sweep_utilities.concat_results(
        job_ids, output_dir, "depth_odom_stats.csv"
    )


def make_value_ranges():
    value_ranges = []
    value_names = []
    steps = 10

    value_ranges.append(np.linspace(10, 100, steps, endpoint=True))
    value_names.append("lk_max_corners")
    return value_ranges, value_names


def make_values_and_parameter_sweep(
    output_dir,
    bag_file,
    config_path,
    robot_config,
    world,
    groundtruth_bagfile,
    rmse_rel_start_time=0,
    rmse_rel_end_time=-1,
):
    output_dir = lu.create_directory(output_dir)
    print(("Output directory for results is {}".format(output_dir)))

    value_ranges, value_names = make_value_ranges()
    parameter_sweep_utilities.save_values(
        value_names, value_ranges, "individual_value_ranges.csv", output_dir
    )

    all_value_combos = parameter_sweep_utilities.make_all_value_combinations(
        value_ranges
    )
    parameter_sweep_utilities.save_values(
        value_names, all_value_combos, "all_value_combos.csv", output_dir
    )

    parameter_sweep(
        all_value_combos,
        value_names,
        output_dir,
        bag_file,
        config_path,
        robot_config,
        world,
        groundtruth_bagfile,
        rmse_rel_start_time,
        rmse_rel_end_time,
    )
    combined_results_file = os.path.join(output_dir, "param_sweep_combined_results.csv")
    value_combos_file = os.path.join(output_dir, "all_value_combos.csv")
    results_pdf_file = os.path.join(output_dir, "param_sweep_results.pdf")
    plot_parameter_sweep_results.create_plots(
        results_pdf_file, combined_results_file, value_combos_file
    )
    return output_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bag_file", help="Full path to bagfile.")
    parser.add_argument("config_path", help="Full path to config path.")
    parser.add_argument("robot_config", help="Relative path to robot config.")
    parser.add_argument("world", help="World being used.")
    parser.add_argument("-g", "--groundtruth-bagfile", default=None)
    parser.add_argument(
        "--directory",
        default=None,
        help="Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.",
    )
    args = parser.parse_args()
    # Default set groundtruth bagfile to normal bagfile
    if not args.groundtruth_bagfile:
        args.groundtruth_bagfile = args.bag_file

    make_values_and_parameter_sweep(
        args.directory,
        args.bag_file,
        args.config_path,
        args.robot_config,
        args.world,
        args.groundtruth_bagfile,
    )
