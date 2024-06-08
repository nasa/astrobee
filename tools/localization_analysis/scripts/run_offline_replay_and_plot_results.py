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
Runs localization for a bagfile using the provided map.
Saves results to an output bagfile and generates a pdf
file with detailed results.
"""

import argparse
import os
import sys

import localization_common.utilities as lu

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument("map_file", help="Map file.")
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Image topic.",
    )
    parser.add_argument(
        "-r", "--robot-config-file", default="bumble.config", help="Robot config file."
    )
    parser.add_argument("-w", "--world", default="iss", help="World (iss or granite).")
    parser.add_argument(
        "-o", "--output-bagfile", default="results.bag", help="Output bagfile."
    )
    parser.add_argument(
        "--generate-image-features",
        dest="use_image_features",
        action="store_false",
        help="Generate image features instead of using image features msgs from bagfile.",
    )
    parser.add_argument("--vio-output-file", default="vio_output.pdf")
    parser.add_argument("--loc-output-file", default="loc_output.pdf")
    parser.add_argument("--vio-results-csv-file", default="vio_results.csv")
    parser.add_argument("--loc-results-csv-file", default="loc_results.csv")
    parser.add_argument("--results-csv-file", default="results.csv")
    parser.add_argument("-g", "--groundtruth-bagfile", default=None)
    parser.add_argument(
        "--directory",
        default=None,
        help="Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print("Bag file " + args.bagfile + " does not exist.")
        sys.exit()
    if not os.path.isfile(args.map_file):
        print("Map file " + args.map_file + " does not exist.")
        sys.exit()
    if os.path.isfile(args.vio_output_file):
        print("VIO output file " + args.vio_output_file + " already exists.")
        sys.exit()
    if os.path.isfile(args.loc_output_file):
        print("Loc output file " + args.loc_output_file + " already exists.")
        sys.exit()
    if os.path.isfile(args.vio_results_csv_file):
        print("VIO results csv file " + args.vio_results_csv_file + " already exists.")
        sys.exit()
    if os.path.isfile(args.loc_results_csv_file):
        print("Loc results csv file " + args.loc_results_csv_file + " already exists.")
        sys.exit()
    if args.groundtruth_bagfile and not os.path.isfile(args.groundtruth_bagfile):
        print("Groundtruth bag " + args.groundtruth_bagfile + " does not exist.")
        sys.exit()

    bagfile = os.path.abspath(args.bagfile)
    map_file = os.path.abspath(args.map_file)

    # Run localizer
    run_offline_replay_command = (
        "rosrun localization_analysis run_offline_replay "
        + bagfile
        + " "
        + map_file
        + " --use-bag-image-feature-msgs "
        + (str(args.use_image_features)).lower()
        + " -o "
        + args.output_bagfile
        + " -r "
        + args.robot_config_file
        + " -w "
        + args.world
        + " -s "
        + args.results_csv_file
    )
    lu.run_command_and_save_output(
        run_offline_replay_command, "run_offline_replay_command.txt"
    )

    # Plot results
    plot_vio_results_command = (
        "rosrun localization_analysis vio_results_plotter.py "
        + args.output_bagfile
        + " --output-file "
        + args.vio_output_file
        + " --results-csv-file "
        + args.vio_results_csv_file
    )
    if args.groundtruth_bagfile:
        plot_vio_results_command += " -g " + args.groundtruth_bagfile
    lu.run_command_and_save_output(
        plot_vio_results_command, "plot_vio_results_command.txt"
    )

    # Plot loc results
    plot_loc_results_command = (
        "rosrun localization_analysis loc_results_plotter.py "
        + args.output_bagfile
        + " --output-file "
        + args.loc_output_file
        + " --results-csv-file "
        + args.loc_results_csv_file
    )
    if args.groundtruth_bagfile:
        plot_loc_results_command += " -g " + args.groundtruth_bagfile
    lu.run_command_and_save_output(
        plot_loc_results_command, "plot_loc_results_command.txt"
    )
