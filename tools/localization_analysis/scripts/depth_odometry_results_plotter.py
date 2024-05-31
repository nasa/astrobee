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
Creates plots for Depth Odometry results compared with groundtruth. 
"""

import argparse
import os
import sys

import matplotlib

import message_reader
import plot_conversions
import results_savers
from multipose_plotter import MultiPosePlotter
from multivector3d_plotter import MultiVector3dPlotter
from timestamped_pose import TimestampedPose

matplotlib.use("pdf")
import csv
import math

import geometry_msgs
import matplotlib.pyplot as plt
import rosbag
from matplotlib.backends.backend_pdf import PdfPages


def plot_depth_odom_results(pdf, results_csv_file, groundtruth_poses, depth_odometries):
    absolute_depth_odom_relative_poses = plot_conversions.absolute_poses_from_relative_poses(
        plot_conversions.poses_from_depth_odometries(depth_odometries),
        groundtruth_poses,
    )
    depth_odom_relative_poses_plotter = MultiPosePlotter(
        "Time (s)", "Position (m)", "Depth Odometry vs. Groundtruth Position", True
    )
    depth_odom_relative_poses_plotter.add_poses(
        "Groundtruth Poses",
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    depth_odom_relative_poses_plotter.add_poses(
        "Depth Odometry Poses", absolute_depth_odom_relative_poses, linestyle="-"
    )
    depth_odom_relative_poses_plotter.plot(pdf)

    num_features_plotter = plot_conversions.num_features_plotter_from_depth_odometries(
        depth_odometries
    )
    num_features_plotter.plot(pdf)

    runtime_plotter = plot_conversions.runtime_plotter_from_depth_odometries(
        depth_odometries
    )
    runtime_plotter.plot(pdf)


# Loads poses from the provided bagfile, generates plots, and saves results to a pdf and csv file.
# The csv file contains results in (TODO: define format).
# The RMSE rel start and end time define the time range for evaluating the RMSE, in case the bag starts or ends with little/uninteresting movement
# that shouldn't be included in the RMSE calculation.
# The groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def load_data_and_create_depth_odom_plots(
    bagfile,
    output_pdf_file,
    results_csv_file="depth_odom_results.csv",
    groundtruth_bagfile=None,
    rmse_rel_start_time=0,
    rmse_rel_end_time=-1,
):
    # Load bagfile with depth_odom results
    bag = rosbag.Bag(bagfile)
    # Load bagfile with groundtruth poses. Assume groundtruth is in the same results bag
    # if no separate bagfile for groundtruth is provided.
    groundtruth_bag = rosbag.Bag(groundtruth_bagfile) if groundtruth_bagfile else bag
    bag_start_time = bag.get_start_time()

    # Load groundtruth poses
    # Use sparse mapping poses as groundtruth.
    groundtruth_poses = []
    message_reader.load_poses(
        groundtruth_poses, "/sparse_mapping/pose", groundtruth_bag, bag_start_time
    )

    depth_odometries = message_reader.load_depth_odometries(
        "/loc/depth/odom", bag, bag_start_time
    )
    bag.close()

    with PdfPages(output_pdf_file) as pdf:
        plot_depth_odom_results(
            pdf, results_csv_file, groundtruth_poses, depth_odometries
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument(
        "--output-file", default="depth_odom_output.pdf", help="Output pdf file."
    )
    parser.add_argument(
        "--results-csv-file",
        default="depth_odom_results.csv",
        help="Output csv file containing results stats.",
    )
    parser.add_argument(
        "-g",
        "--groundtruth-bagfile",
        default=None,
        help="Optional bagfile containing groundtruth poses to use as a comparison for poses in the input bagfile. If none provided, sparse mapping poses are used as groundtruth from the input bagfile if available.",
    )
    parser.add_argument(
        "--rmse-rel-start-time",
        type=float,
        default=0,
        help="Optional start time for plots.",
    )
    parser.add_argument(
        "--rmse-rel-end-time",
        type=float,
        default=-1,
        help="Optional end time for plots.",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()
    if args.groundtruth_bagfile and not os.path.isfile(args.groundtruth_bagfile):
        print(("Groundtruth Bag file " + args.groundtruth_bagfile + " does not exist."))
        sys.exit()
    load_data_and_create_depth_odom_plots(
        args.bagfile,
        args.output_file,
        args.results_csv_file,
        args.groundtruth_bagfile,
        args.rmse_rel_start_time,
        args.rmse_rel_end_time,
    )