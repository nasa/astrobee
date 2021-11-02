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
import sys

import utilities

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument("map_file", help="Map file.")
    parser.add_argument("config_path", help="Full path to config path.")
    parser.add_argument("-r",
                        "--robot-config",
                        default='config/robots/bumble.config',
                        help="Relative path to robot config.")
    parser.add_argument("-i", "--image-topic", default='/mgt/img_sampler/nav_cam/image_record', help="Image topic.")
    parser.add_argument("-o", "--output-bagfile", default='results.bag', help="Output bagfile.")
    parser.add_argument('-w', '--world', default='iss')
    parser.add_argument(
        "--generate-image-features",
        dest="use_image_features",
        action="store_false",
        help="Use image features msgs from bagfile or generate features from images.",
    )
    parser.add_argument("--output-file", default="output.pdf")
    parser.add_argument("--output-csv-file", default="results.csv")
    parser.add_argument("-g", "--groundtruth-bagfile", default=None)
    parser.add_argument(
        "--directory",
        default=None,
        help=
        "Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print('Bag file ' + args.bagfile + ' does not exist.')
        sys.exit()
    if not os.path.isfile(args.map_file):
        print('Map file ' + args.map_file + ' does not exist.')
        sys.exit()
    if os.path.isfile(args.output_file):
        print('Output file ' + args.output_file + ' already exist.')
        sys.exit()
    if os.path.isfile(args.output_csv_file):
        print('Output csv file ' + args.output_csv_file + ' already exist.')
        sys.exit()
    if args.groundtruth_bagfile and not os.path.isfile(args.groundtruth_bagfile):
        print('Groundtruth bag ' + args.groundtruth_bagfile + ' does not exist.')
        sys.exit()

    bagfile = os.path.abspath(args.bagfile)
    map_file = os.path.abspath(args.map_file)

    # Run localizer
    run_graph_bag_command = 'rosrun graph_bag run_graph_bag ' + bagfile + ' ' + map_file + ' ' + args.config_path + ' -r ' + args.robot_config + ' -w ' + args.world + ' --use-image-features ' + str(
        args.use_image_features) + ' -o ' + args.output_bagfile
    utilities.run_command_and_save_output(run_graph_bag_command, 'run_graph_bag_command.txt')

    # Plot results
    plot_results_command = 'rosrun graph_bag plot_results_main.py ' + bagfile + ' --output-file ' + args.output_file + ' --output-csv-file ' + args.output_csv_file
    if args.groundtruth_bagfile:
        plot_results_command += ' -g ' + args.groundtruth_bagfile
    utilities.run_command_and_save_output(plot_results_command, 'plot_results_command.txt')
