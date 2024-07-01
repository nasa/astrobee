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
"""
Runs a parameter sweep for the graph localizer or graph vio using the provided bagfile.
Parameters to sweep on are set in the make_value_ranges function and applied to 
the graph_localizer/vio.config file.
All combinations of provided parameters are used for the sweep and the results
are plotted for various RMSEs. 
"""


import argparse
import csv
import itertools
import math
import multiprocessing
import os

import numpy as np
import rospkg

import config_creator
import localization_common.utilities as lu
import parameter_sweep_results_plotter
import parameter_sweep_utilities


# Create new config file with substituted values
def make_config_file(config_filename, new_output_dir, config_path, values, value_names):
    config_filepath = os.path.join(config_path, config_filename)
    new_config_filepath = os.path.join(new_output_dir, config_filename)
    config_creator.make_config(
        values, value_names, config_filepath, new_config_filepath
    )


def create_results_plots(output_dir, prefix):
    combined_results_file = os.path.join(
        output_dir, prefix + "_param_sweep_combined_results.csv"
    )
    value_combos_file = os.path.join(output_dir, "all_value_combos.csv")
    results_pdf_file = os.path.join(output_dir, prefix + "_param_sweep_results.pdf")
    parameter_sweep_results_plotter.create_plots(
        results_pdf_file, combined_results_file, value_combos_file
    )


# Run graph localizer/vio with values.
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
    map_file,
    image_topic,
    robot_config,
    world,
    use_bag_image_msgs,
    groundtruth_bagfile,
):
    new_output_dir_prefix = os.path.join(output_dir, str(job_id))
    new_output_dir = os.path.join(new_output_dir_prefix, "localization")
    os.makedirs(new_output_dir)
    config_path = os.path.join(
        rospkg.RosPack().get_path("astrobee"), "config/localization"
    )
    make_config_file(
        "graph_localizer.config", new_output_dir, config_path, values, value_names
    )
    make_config_file(
        "graph_vio.config", new_output_dir, config_path, values, value_names
    )
    make_config_file(
        "imu_integrator.config", new_output_dir, config_path, values, value_names
    )
    make_config_file(
        "imu_filter.config", new_output_dir, config_path, values, value_names
    )
    output_bag = os.path.join(new_output_dir, "results.bag")
    output_stats_file = os.path.join(new_output_dir, "graph_stats.csv")
    graph_config_prefix = new_output_dir_prefix + "/"
    run_command = (
        "rosrun localization_analysis run_offline_replay "
        + bag_file
        + " "
        + map_file
        + " -o "
        + output_bag
        + " -s "
        + output_stats_file
        + " -r "
        + robot_config
        + " -w "
        + world
        + " -g "
        + graph_config_prefix
        + " -f "
        + str(use_bag_image_msgs)
    )
    if image_topic is not None:
        run_command += " -i " + image_topic
    os.system(run_command)

    # Use stored sparse mapping poses from output bag if no groundtruth bag provided
    if not groundtruth_bagfile:
        groundtruth_bagfile = output_bag

    # Plot Loc results
    output_pdf_file = os.path.join(
        new_output_dir_prefix, str(job_id) + "_loc_output.pdf"
    )
    results_csv_file = os.path.join(new_output_dir_prefix, "loc_graph_stats.csv")
    plot_command = (
        "rosrun localization_analysis loc_results_plotter.py "
        + output_bag
        + " --output-file "
        + output_pdf_file
        + " --results-csv-file "
        + results_csv_file
        + " -g "
        + groundtruth_bagfile
    )
    os.system(plot_command)

    # Plot VIO results
    output_pdf_file = os.path.join(
        new_output_dir_prefix, str(job_id) + "_vio_output.pdf"
    )
    results_csv_file = os.path.join(new_output_dir_prefix, "vio_graph_stats.csv")
    plot_command = (
        "rosrun localization_analysis vio_results_plotter.py "
        + output_bag
        + " --output-file "
        + output_pdf_file
        + " --results-csv-file "
        + results_csv_file
        + " -g "
        + groundtruth_bagfile
    )
    os.system(plot_command)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def test_values_helper(zipped_vals):
    return test_values(*zipped_vals)


# Create parallel jobs for each generated parameter sweep config set
def parameter_sweep(
    all_value_combos,
    value_names,
    output_dir,
    bag_file,
    map_file,
    image_topic,
    robot_config,
    world,
    use_bag_image_msgs,
    groundtruth_bagfile,
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
                itertools.repeat(map_file),
                itertools.repeat(image_topic),
                itertools.repeat(robot_config),
                itertools.repeat(world),
                itertools.repeat(use_bag_image_msgs),
                itertools.repeat(groundtruth_bagfile),
            )
        ),
    )
    parameter_sweep_utilities.concat_results(
        job_ids, output_dir, "loc_graph_stats.csv", "loc"
    )
    parameter_sweep_utilities.concat_results(
        job_ids, output_dir, "vio_graph_stats.csv", "vio"
    )


# Create values for defined config options to sweep on
def make_value_ranges():
    value_ranges = []
    value_names = []
    steps = 10

    # tune num smart factors
    # value_ranges.append(np.logspace(-3, -5, steps, endpoint=True))
    # value_names.append("ii_accel_bias_sigma")
    value_ranges.append(np.logspace(-3, -1, steps, endpoint=True))
    value_names.append("gv_fa_do_point_noise_scale")
    # value_ranges.append(np.logspace(-3, -7, steps, endpoint=True))
    # value_names.append("ii_bias_acc_omega_int")

    # value_ranges.append(np.logspace(0, -5, steps, endpoint=True))
    # value_names.append("gv_fa_vo_retriangulation_threshold")

    # q_gyro
    # .001 -> 2 deg
    # q_gyro_degrees_range = np.logspace(-3, .3, steps, endpoint=True)
    # q_gyro_squared_rads_range = [math.radians(deg)**2 for deg in q_gyro_degrees_range]
    # value_ranges.append(q_gyro_squared_rads_range)
    # value_names.append('q_gyro')

    return value_ranges, value_names


# Perform the parameter sweep using defined range of values
def make_values_and_parameter_sweep(
    output_dir,
    bag_file,
    map_file,
    image_topic,
    robot_config,
    world,
    use_bag_image_msgs,
    groundtruth_bagfile,
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
        map_file,
        image_topic,
        robot_config,
        world,
        use_bag_image_msgs,
        groundtruth_bagfile,
    )

    create_results_plots(output_dir, "loc")
    create_results_plots(output_dir, "vio")
    return output_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bag_file", help="Full path to bagfile.")
    parser.add_argument("map_file", help="Full path to map file.")
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Image topic.",
    )
    parser.add_argument(
        "-r",
        "--robot-config",
        default="bumble.config",
        help="Relative path to robot config.",
    )
    parser.add_argument("-w", "--world", default="iss", help="World being used.")
    parser.add_argument(
        "--use-bag-image-feature-msgs",
        dest="use_bag_image_feature_msgs",
        action="store_false",
        help="Use image features msgs from bagfile or generate features from images.",
    )

    parser.add_argument("-g", "--groundtruth-bagfile", default=None)
    parser.add_argument(
        "--directory",
        default=None,
        help="Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.",
    )
    args = parser.parse_args()

    make_values_and_parameter_sweep(
        args.directory,
        args.bag_file,
        args.map_file,
        args.image_topic,
        args.robot_config,
        args.world,
        args.use_bag_image_feature_msgs,
        args.groundtruth_bagfile,
    )
