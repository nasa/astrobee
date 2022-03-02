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
Generates groundtruth for bagfiles in parallel using a provided config script.
See make_groundtruth.py for more details on the groundtruth map, bagfile, and
pdfs created in this process.
"""

import argparse
import csv
import itertools
import multiprocessing
import os
import sys

import multiprocessing_helpers
import utilities


class GroundtruthParams(object):
    def __init__(
        self,
        bagfile,
        base_surf_map,
        maps_directory,
        loc_map,
        config_path,
        world,
        image_topic,
        robot_name,
        use_image_features,
    ):
        self.bagfile = bagfile
        self.base_surf_map = base_surf_map
        self.maps_directory = maps_directory
        self.loc_map = loc_map
        self.config_path = config_path
        self.world = world
        self.image_topic = image_topic
        self.robot_name = robot_name
        self.use_image_features = use_image_features


def load_params(param_file):
    groundtruth_params_list = []
    with open(param_file) as param_csvfile:
        reader = csv.reader(param_csvfile, delimiter=" ")
        for row in reader:
            groundtruth_params_list.append(
                GroundtruthParams(
                    row[0],
                    row[1],
                    row[2],
                    row[3],
                    row[4],
                    row[5],
                    row[6],
                    row[7],
                    row[8],
                )
            )

    return groundtruth_params_list


def check_params(groundtruth_params_list):
    for params in groundtruth_params_list:
        if not os.path.isfile(params.bagfile):
            print(("Bagfile " + params.bagfile + " does not exist."))
            sys.exit()
        if not os.path.isfile(params.base_surf_map):
            print(("Base surf map " + params.base_surf_map + " does not exist."))
            sys.exit()
        if not os.path.isdir(params.maps_directory):
            print(("Maps directory " + params.maps_directory + " does not exist."))
            sys.exit()
        if not os.path.isfile(params.loc_map):
            print(("Loc map " + params.loc_map + " does not exist."))
            sys.exit()


# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@multiprocessing_helpers.full_traceback
def run_groundtruth(params):
    output_directory = utilities.basename(params.bagfile) + "_groundtruth"
    groundtruth_command = (
        "rosrun localization_analysis make_groundtruth.py "
        + params.bagfile
        + " "
        + params.base_surf_map
        + " "
        + params.maps_directory
        + " "
        + params.loc_map
        + " "
        + params.config_path
        + " -o "
        + output_directory
        + " -w "
        + params.world
        + " -i "
        + params.image_topic
        + " -r "
        + params.robot_name
    )
    if not bool(params.use_image_features):
        groundtruth_command += " --generate-image-features"

    output_file = utilities.basename(params.bagfile) + "_groundtruth.txt"
    utilities.run_command_and_save_output(groundtruth_command, output_file)


def groundtruth_sweep(config_file, num_processes):
    groundtruth_params_list = load_params(config_file)
    check_params(groundtruth_params_list)
    pool = multiprocessing.Pool(num_processes)
    pool.map(run_groundtruth, groundtruth_params_list)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "config_file",
        help="Config file containing arguments for each job to run.  Should be formatted with one job per line and using a single space between each argument.  Arguments for a job in order are: bagfile base_surf_map maps_directory loc_map config_path world image_topic robot_name use_image_features.  See make_groundtruth.py description for more details on each argument.",
    )
    parser.add_argument("-o", "--output-directory", default="groundtruth_sweep")
    parser.add_argument(
        "-p",
        "--num-processes",
        type=int,
        default=10,
        help="Number of concurrent processes to run, where each groundtruth creation job is assigned to one process.",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.config_file):
        print(("Config file " + args.config_file + " does not exist."))
        sys.exit()
    if os.path.isdir(args.output_directory):
        print(("Output directory " + args.output_directory + " already exists."))
        sys.exit()
    config_file = os.path.abspath(args.config_file)

    os.mkdir(args.output_directory)
    os.chdir(args.output_directory)

    groundtruth_sweep(config_file, args.num_processes)
