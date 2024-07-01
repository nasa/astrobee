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
Creates plots for Localization results. Adds plots for poses and detectected
features counts. Optionally plots groundtruth poses along with other pose data if groundtruth is provided.
"""

import argparse
import os
import sys

import matplotlib

import message_reader
import plot_conversions
import plotters
import results_savers

matplotlib.use("pdf")
import csv
import math

import geometry_msgs
import matplotlib.pyplot as plt
import rosbag
from matplotlib.backends.backend_pdf import PdfPages


def plot_loc_results(
    pdf,
    results_csv_file,
    groundtruth_poses,
    graph_loc_states,
    extrapolated_loc_states,
    ar_tag_poses,
    imu_accelerations,
):
    poses_plotter = plotters.MultiPosePlotter(
        "Time (s)", "Position (m)", "Loc vs. Groundtruth Position", True
    )
    poses_plotter.add_poses(
        "Groundtruth Poses",
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )

    graph_loc_poses = plot_conversions.poses_from_graph_loc_states(graph_loc_states)
    poses_plotter.add_poses("Graph Loc Poses", graph_loc_poses, linestyle="-")

    poses_plotter.add_poses(
        "AR Tag Poses",
        ar_tag_poses,
        linestyle="None",
        marker="x",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    poses_plotter.plot(pdf)
    results_savers.save_rmse(
        graph_loc_poses, groundtruth_poses, results_csv_file, pdf, "Graph Loc Poses"
    )

    if extrapolated_loc_states:
        extrapolated_poses_plotter = plotters.MultiPosePlotter(
            "Time (s)",
            "Position (m)",
            "Extrapolated Loc vs. Groundtruth Position",
            True,
        )
        extrapolated_poses_plotter.add_poses(
            "Groundtruth Poses",
            groundtruth_poses,
            linestyle="None",
            marker="o",
            markeredgewidth=0.1,
            markersize=1.5,
        )

        extrapolated_loc_poses = plot_conversions.poses_from_extrapolated_loc_states(
            extrapolated_loc_states
        )
        extrapolated_poses_plotter.add_poses(
            "Extrapolated Loc Poses", extrapolated_loc_poses, linestyle="-"
        )
        extrapolated_poses_plotter.plot(pdf)
        results_savers.save_rmse(
            extrapolated_loc_poses,
            groundtruth_poses,
            results_csv_file,
            pdf,
            "Extrapolated Loc Poses",
        )

        extrapolated_velocities_plotter = plotters.MultiVector3dPlotter(
            "Time (s)", "Velocity (m/s)", "Extrapolated Velocities", True
        )
        extrapolated_velocity_plotter = (
            plot_conversions.velocity_plotter_from_extrapolated_loc_states(
                extrapolated_loc_states
            )
        )
        extrapolated_velocities_plotter.add(extrapolated_velocity_plotter)
        extrapolated_velocities_plotter.plot(pdf)

        extrapolated_integrated_velocity_poses = plot_conversions.absolute_poses_from_integrated_extrapolated_loc_state_velocities(
            extrapolated_loc_states, groundtruth_poses
        )
        extrapolated_integrated_velocity_poses_plotter = plotters.MultiPosePlotter(
            "Time (s)",
            "Position (m)",
            "Extrapolated Loc Integrated Velocities vs. Groundtruth Position",
            True,
        )
        extrapolated_integrated_velocity_poses_plotter.add_poses(
            "Groundtruth Poses",
            groundtruth_poses,
            linestyle="None",
            marker="o",
            markeredgewidth=0.1,
            markersize=1.5,
        )
        extrapolated_integrated_velocity_poses_plotter.add_poses(
            "Extrapolted Loc Integrated Velocity Poses",
            extrapolated_integrated_velocity_poses,
            linestyle="-",
        )
        extrapolated_integrated_velocity_poses_plotter.plot_positions(pdf)
        results_savers.save_rmse(
            extrapolated_integrated_velocity_poses,
            groundtruth_poses,
            results_csv_file,
            pdf,
            "Extrapolated Integrated Velocity Poses",
        )

        extrapolated_accelerations_plotter = plotters.MultiVector3dPlotter(
            "Time (s)", "Acceleration (m/s^2)", "Bias Corrected Accelerations", True
        )
        extrapolated_acceleration_plotter = (
            plot_conversions.acceleration_plotter_from_extrapolated_loc_states(
                extrapolated_loc_states
            )
        )
        extrapolated_accelerations_plotter.add(extrapolated_acceleration_plotter)
        extrapolated_accelerations_plotter.plot(pdf)

    if imu_accelerations:
        imu_accelerations_plotter = plotters.MultiVector3dPlotter(
            "Time (s)", "Acceleration (m/s^2)", "Raw IMU Accelerations", True
        )
        imu_acceleration_plotter = (
            plot_conversions.acceleration_plotter_from_imu_accelerations(
                imu_accelerations
            )
        )
        imu_accelerations_plotter.add(imu_acceleration_plotter)
        imu_accelerations_plotter.plot(pdf)

    ml_count_plotter = plot_conversions.ml_feature_count_plotter_from_graph_loc_states(
        graph_loc_states
    )
    ml_count_plotter.plot(pdf)

    ar_count_plotter = plot_conversions.ar_feature_count_plotter_from_graph_loc_states(
        graph_loc_states
    )
    ar_count_plotter.plot(pdf)

    ml_num_pose_factors_plotter = (
        plot_conversions.ml_pose_factor_count_plotter_from_graph_loc_states(
            graph_loc_states
        )
    )
    ml_num_pose_factors_plotter.plot(pdf)

    ml_num_projection_factors_plotter = (
        plot_conversions.ml_projection_factor_count_plotter_from_graph_loc_states(
            graph_loc_states
        )
    )
    ml_num_projection_factors_plotter.plot(pdf)

    num_states_plotter = plot_conversions.num_states_plotter_from_states(
        graph_loc_states
    )
    num_states_plotter.plot(pdf)

    duration_plotter = plot_conversions.duration_plotter_from_states(graph_loc_states)
    duration_plotter.plot(pdf)

    optimization_time_plotter = plot_conversions.optimization_time_plotter_from_states(
        graph_loc_states
    )
    optimization_time_plotter.plot(pdf)

    update_time_plotter = plot_conversions.update_time_plotter_from_states(
        graph_loc_states
    )
    update_time_plotter.plot(pdf)

    optimization_iterations_plotter = (
        plot_conversions.optimization_iterations_plotter_from_states(graph_loc_states)
    )
    optimization_iterations_plotter.plot(pdf)


# Loads poses from the provided bagfile, generates plots, and saves results to a pdf and csv file.
# The csv file contains results in (TODO: define format).
# The RMSE rel start and end time define the time range for evaluating the RMSE, in case the bag starts or ends with little/uninteresting movement
# that shouldn't be included in the RMSE calculation.
# The groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def load_data_and_create_loc_plots(
    bagfile,
    output_pdf_file,
    results_csv_file="loc_results.csv",
    groundtruth_bagfile=None,
    rmse_rel_start_time=0,
    rmse_rel_end_time=-1,
):
    # Load bagfile with localization results
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

    # Load graph localization states
    graph_loc_states = []
    message_reader.load_graph_loc_states(
        graph_loc_states, "/graph_loc/state", bag, bag_start_time
    )

    # Load extrapolated localization states
    extrapolated_loc_states = []
    message_reader.load_extrapolated_loc_states(
        extrapolated_loc_states, "/gnc/ekf", bag, bag_start_time
    )

    # Load AR Tag poses
    ar_tag_poses = []
    message_reader.load_poses(ar_tag_poses, "/ar_tag/pose", bag, bag_start_time)

    # Load IMU data
    imu_accelerations = []
    message_reader.load_imu_accelerations(
        imu_accelerations, "/hw/imu", bag, bag_start_time
    )
    bag.close()

    with PdfPages(output_pdf_file) as pdf:
        plot_loc_results(
            pdf,
            results_csv_file,
            groundtruth_poses,
            graph_loc_states,
            extrapolated_loc_states,
            ar_tag_poses,
            imu_accelerations,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument(
        "--output-file", default="loc_output.pdf", help="Output pdf file."
    )
    parser.add_argument(
        "--results-csv-file",
        default="loc_results.csv",
        help="Output csv file containing localization stats.",
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
    load_data_and_create_loc_plots(
        args.bagfile,
        args.output_file,
        args.results_csv_file,
        args.groundtruth_bagfile,
        args.rmse_rel_start_time,
        args.rmse_rel_end_time,
    )
