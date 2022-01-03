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

import argparse
import os

import parameter_sweep
import utilities


def bag_and_parameter_sweep(bag_files, output_dir, map_file, image_topic, gnc_config):
    for bag_file in bag_files:
        # Save parameter sweep output in different directory for each bagfile, name directory using bagfile
        bag_name_prefix = os.path.splitext(os.path.basename(bag_file))[0]
        bag_output_dir = os.path.join(output_dir, bag_name_prefix)
        parameter_sweep.make_values_and_parameter_sweep(
            bag_output_dir, bag_file, map_file, image_topic, gnc_config
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_file", help="Full path to map file.")
    parser.add_argument("gnc_config", help="Full path to gnc config file.")
    parser.add_argument(
        "-d",
        "--bag_directory",
        default=None,
        help="Full path to directory containing bag files.",
    )
    parser.add_argument(
        "-o",
        "--output_directory",
        default=None,
        help="Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.",
    )

    parser.add_argument(
        "-r",
        "--robot_name",
        metavar="ROBOT",
        help="Specify the robot to use (just name, not path).",
    )
    parser.add_argument(
        "-i",
        "--image_topic",
        dest="image_topic",
        default=None,
        help="Use specified image topic.",
    )
    parser.add_argument(
        "--save_stats", action="store_true", help="Save stats to csv file."
    )
    parser.add_argument(
        "--make_plots", type=bool, default=True, help="Make pdf of plots of results."
    )

    args, args_unkonown = parser.parse_known_args()

    bag_directory = args.bag_directory
    if bag_directory == None:
        bag_directory = os.getcwd()

    if not os.path.exists(bag_directory):
        print(("Bag directory {} does not exist.".format(bag_directory)))
        exit()
    bag_files = utilities.get_files(bag_directory, "*.bag")
    print(("Found {} bag files in {}.".format(len(bag_files), bag_directory)))

    output_dir = utilities.create_directory(args.output_directory)
    print(("Output directory for results is {}".format(output_dir)))

    bag_and_parameter_sweep(
        bag_files, output_dir, args.map_file, args.image_topic, args.gnc_config
    )
