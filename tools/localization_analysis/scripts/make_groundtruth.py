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
Generates the groundtruth map and groundtruth bagfile containing groundtruth 
localization estimates for a given input bagfile.
Also tests the input bagfile against a provided localization map and plots the 
results compared with the newly created groundtruth.
"""

import argparse
import os
import shutil
import sys

import make_map

import localization_common.utilities as lu

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile to generate groundtruth for.")
    parser.add_argument(
        "base_surf_map",
        help="Existing map to use as basis for groundtruth.  Should largely overlap area covered in input bagfile.",
    )
    parser.add_argument(
        "maps_directory",
        help="Location of images used for each bagfile use to generate base_surf_map.",
    )
    parser.add_argument(
        "loc_map", help="Localization map for bagfile to test localization performance."
    )
    parser.add_argument("config_path", help="Full path to config path.")
    parser.add_argument(
        "-o", "--output-directory", default="groundtruth_creation_output"
    )
    parser.add_argument("-w", "--world", default="iss")
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Image topic.",
    )
    parser.add_argument("-r", "--robot-name", default="bumble")
    parser.add_argument(
        "-m",
        "--map-name",
        default=None,
        help="Prefix for generated map names. Defaults to bagfile name.",
    )
    parser.add_argument(
        "--generate-image-features",
        dest="use_image_features",
        action="store_false",
        help="Use image features msgs from bagfile or generate features from images.",
    )

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print("Bag file " + args.bagfile + " does not exist.")
        sys.exit()
    if not os.path.isfile(args.base_surf_map):
        print("Base surf map " + args.base_surf_map + " does not exist.")
        sys.exit()
    if not os.path.isfile(args.loc_map):
        print("Loc map " + args.loc_map + " does not exist.")
        sys.exit()
    if not os.path.isdir(args.maps_directory):
        print("Maps directory " + args.maps_directory + " does not exist.")
        sys.exit()
    if os.path.isdir(args.output_directory):
        print("Output directory " + args.output_directory + " already exists.")
        sys.exit()

    bagfile = os.path.abspath(args.bagfile)
    base_surf_map = os.path.abspath(args.base_surf_map)
    maps_directory = os.path.abspath(args.maps_directory)

    os.mkdir(args.output_directory)
    os.chdir(args.output_directory)

    map_name = args.map_name
    bag_prefix = lu.basename(bagfile)
    if not args.map_name:
        map_name = bag_prefix + "_groundtruth"

    make_map.make_map(
        bagfile, map_name, args.world, args.robot_name, base_surf_map, maps_directory
    )

    robot_config = "config/robots/" + args.robot_name + ".config"
    groundtruth_bag = map_name + ".bag"
    groundtruth_map_file = map_name + ".brisk.vocabdb.map"
    groundtruth_pdf = "groundtruth.pdf"
    groundtruth_csv = "groundtruth.csv"
    make_groundtruth_command = (
        "rosrun localization_analysis run_graph_bag_and_plot_results.py "
        + bagfile
        + " "
        + groundtruth_map_file
        + " "
        + args.config_path
        + " -i "
        + args.image_topic
        + " -r "
        + robot_config
        + " -w "
        + args.world
        + " -o "
        + groundtruth_bag
        + " --output-file "
        + groundtruth_pdf
        + " --output-csv-file "
        + groundtruth_csv
        + " --generate-image-features"
    )
    lu.run_command_and_save_output(make_groundtruth_command, "make_groundtruth.txt")
    os.rename("run_graph_bag_command.txt", "groundtruth_run_graph_bag_command.txt")

    loc_results_bag = bag_prefix + "_results.bag"
    loc_pdf = "loc_results.pdf"
    loc_csv = "loc_results.csv"
    get_loc_results_command = (
        "rosrun localization_analysis run_graph_bag_and_plot_results.py "
        + bagfile
        + " "
        + args.loc_map
        + " "
        + args.config_path
        + " -i "
        + args.image_topic
        + " -r "
        + robot_config
        + " -w "
        + args.world
        + " -o "
        + loc_results_bag
        + " --output-file "
        + loc_pdf
        + " --output-csv-file "
        + loc_csv
        + " -g "
        + groundtruth_bag
    )
    if not args.use_image_features:
        get_loc_results_command += " --generate-image-features"
    lu.run_command_and_save_output(get_loc_results_command, "get_loc_results.txt")
    os.rename("run_graph_bag_command.txt", "loc_run_graph_bag_command.txt")
