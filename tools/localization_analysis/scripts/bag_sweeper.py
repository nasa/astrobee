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
The bag sweep tool runs offline replay in parallel on multiple bag files.  It takes a config file with bag names, map names, and robot configs and produces pdfs and result bagfiles for each entry.
"""


import argparse
import csv
import itertools
import multiprocessing
import os
import sys

import bag_sweep_results_plotter 
import localization_common.utilities as lu
import results_averager 


class OfflineReplayParams(object):
    def __init__(
        self,
        bagfile,
        map_file,
        image_topic,
        robot_config_file,
        world,
        use_bag_image_feature_msgs,
        groundtruth_bagfile,
    ):
        self.bagfile = bagfile
        self.map_file = map_file
        self.image_topic = image_topic
        self.robot_config_file = robot_config_file
        self.world = world
        self.use_bag_image_feature_msgs = use_bag_image_feature_msgs
        self.groundtruth_bagfile = groundtruth_bagfile


# Load params from the config file
def load_params(param_file):
    offline_replay_params_list = []
    with open(param_file) as param_csvfile:
        reader = csv.reader(param_csvfile, delimiter=" ")
        for row in reader:
            offline_replay_params_list.append(
                OfflineReplayParams(
                    row[0],
                    row[1],
                    row[2],
                    row[3],
                    row[4],
                    row[5],
                    row[6],
                )
            )

    return offline_replay_params_list


# Combine results in a single csv file for plotting later
def combine_results_in_csv_file(offline_replay_params, output_dir):
    # Don't save this as *stats.csv otherwise it will be including when averaging bag results in average_results.py
    combined_results_csv_file = os.path.join(output_dir, "bag_sweep_stats_combined.csv")
    output_csv_files = []
    bag_names = []
    for params in offline_replay_params:
        bag_name = os.path.splitext(os.path.basename(params.bagfile))[0]
        bag_names.append(bag_name)
        output_csv_files.append(os.path.join(output_dir, bag_name + "_stats.csv"))
    combined_dataframe = results_averager.combined_results(output_csv_files)
    combined_dataframe.insert(0, "Bag", bag_names)
    combined_dataframe.to_csv(combined_results_csv_file, index=False)
    return combined_results_csv_file

# Ensure files in params exist
def check_params(offline_replay_params_list):
    for params in offline_replay_params_list:
        if not os.path.isfile(params.bagfile):
            print(("Bagfile " + params.bagfile + " does not exist."))
            sys.exit()
        if not os.path.isfile(params.map_file):
            print(("Map file " + params.map_file + " does not exist."))
            sys.exit()
        if not os.path.isfile(params.groundtruth_bagfile):
            print(("Bagfile " + params.groundtruth_bagfile + " does not exist."))
            sys.exit()


# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@lu.full_traceback
def run_offline_replay(params, output_dir):
    bag_name = os.path.splitext(os.path.basename(params.bagfile))[0]
    output_bag_path = os.path.join(output_dir, bag_name + "_results.bag")
    output_csv_file = os.path.join(output_dir, bag_name + "_stats.csv")
    run_command = (
        "rosrun localization_analysis run_offline_replay "
        + params.bagfile
        + " "
        + params.map_file
        + " -i "
        + params.image_topic
        + " -o "
        + output_bag_path
        + " -r "
        + params.robot_config_file
        + " -w "
        + params.world
        + " -s "
        + output_csv_file
        + " -f "
        + params.use_bag_image_feature_msgs
    )
    os.system(run_command)
    output_pdf_file = os.path.join(output_dir, bag_name + "_output.pdf")
    plot_command = (
        "rosrun localization_analysis plot_results.py "
        + output_bag_path
        + " --output-file "
        + output_pdf_file
        + " --output-csv-file "
        + output_csv_file
        + " -g "
        + params.groundtruth_bagfile
    )
    os.system(plot_command)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def run_offline_replay_helper(zipped_vals):
    return run_offline_replay(*zipped_vals)


# Perform bag sweep using config file, save results to output directory.
def bag_sweep(config_file, output_dir):
    offline_replay_params_list = load_params(config_file)
    check_params(offline_replay_params_list)
    num_processes = 15
    pool = multiprocessing.Pool(num_processes)
    # izip arguments so we can pass as one argument to pool worker
    pool.map(
        run_offline_replay_helper,
        list(zip(offline_replay_params_list, itertools.repeat(output_dir))),
    )
    combined_results_csv_file = combine_results_in_csv_file(
        offline_replay_params_list, output_dir
    )
    output_file = os.path.join(output_dir, "bag_sweep_results.pdf")
    bag_sweep_results_plotter.create_plot(output_file, combined_results_csv_file)


if __name__ == "__main__":

    class Formatter(
        argparse.RawTextHelpFormatter, argparse.RawDescriptionHelpFormatter
    ):
        pass

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=Formatter)
    parser.add_argument(
        "config_file",
        help="Config file containing bag names, map names, image topics, robot config, and world.  A new line should be used for each bagfile.  Example:\n /home/bag_name.bag /home/map_name.map /mgt/img_sampler/nav_cam/image_record bumble.config iss false \n /home/bag_name_2.bag /home/map_name.map /mgt/img_sampler/nav_cam/image_record bumble.config iss false",
    )
    parser.add_argument(
        "output_dir", help="Output directory where results files are saved."
    )
    args = parser.parse_args()
    if not os.path.isfile(args.config_file):
        print(("Config file " + args.config_file + " does not exist."))
        sys.exit()
    if os.path.isdir(args.output_dir):
        print(("Output directory " + args.output_dir + " already exists."))
        sys.exit()
    os.makedirs(args.output_dir)

    bag_sweep(args.config_file, args.output_dir)
