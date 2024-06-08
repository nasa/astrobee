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
Creates plots for VIO results. Adds plots for poses, velocities, IMU bias values, 
integrated velocities forming poses, and detectected
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


def plot_vio_results(
    pdf,
    results_csv_file,
    groundtruth_poses,
    graph_vio_states,
    imu_bias_extrapolated_poses,
    depth_odometries,
):
    poses_plotter = plotters.MultiPosePlotter(
        "Time (s)", "Position (m)", "Graph vs. Groundtruth Position", True
    )
    poses_plotter.add_poses(
        "Groundtruth Poses",
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )

    graph_vio_poses = plot_conversions.adjusted_graph_vio_poses_from_graph_vio_states(
        graph_vio_states, groundtruth_poses
    )
    poses_plotter.add_poses("Graph VIO Poses", graph_vio_poses, linestyle="-")
    poses_plotter.plot(pdf)
    results_savers.save_rmse(
        graph_vio_poses, groundtruth_poses, results_csv_file, pdf, "Graph VIO Poses"
    )

    velocities_plotter = plotters.MultiVector3dPlotter(
        "Time (s)", "Velocity (m/s)", "Graph VIO Velocities", True
    )
    graph_vio_velocity_plotter = (
        plot_conversions.velocity_plotter_from_graph_vio_states(graph_vio_states)
    )
    velocities_plotter.add(graph_vio_velocity_plotter)
    velocities_plotter.plot(pdf)

    accel_bias_plotters = plotters.MultiVector3dPlotter(
        "Time (s)", "Accelerometer Bias (m/s^2)", "Graph VIO Accel. Biases", True
    )
    graph_vio_accel_bias_plotter = (
        plot_conversions.accel_bias_plotter_from_graph_vio_states(graph_vio_states)
    )
    accel_bias_plotters.add(graph_vio_accel_bias_plotter)
    accel_bias_plotters.plot(pdf)

    gyro_bias_plotters = plotters.MultiVector3dPlotter(
        "Time (s)", "Gyro Bias (rad/s^2)", "Graph VIO Gyro. Biases", True
    )
    graph_vio_gyro_bias_plotter = (
        plot_conversions.gyro_bias_plotter_from_graph_vio_states(graph_vio_states)
    )
    gyro_bias_plotters.add(graph_vio_gyro_bias_plotter)
    gyro_bias_plotters.plot(pdf)

    of_count_plotter = (
        plot_conversions.optical_flow_feature_count_plotter_from_graph_vio_states(
            graph_vio_states
        )
    )
    of_count_plotter.plot(pdf)

    of_num_factors_plotter = (
        plot_conversions.optical_flow_factor_count_plotter_from_graph_vio_states(
            graph_vio_states
        )
    )
    of_num_factors_plotter.plot(pdf)

    depth_num_factors_plotter = (
        plot_conversions.depth_factor_count_plotter_from_graph_vio_states(
            graph_vio_states
        )
    )
    depth_num_factors_plotter.plot(pdf)

    integrated_velocity_poses = (
        plot_conversions.absolute_poses_from_integrated_graph_vio_state_velocities(
            graph_vio_states, groundtruth_poses
        )
    )
    integrated_velocity_poses_plotter = plotters.MultiPosePlotter(
        "Time (s)",
        "Position (m)",
        "Integrated Velocities vs. Groundtruth Position",
        True,
    )
    integrated_velocity_poses_plotter.add_poses(
        "Groundtruth Poses",
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    integrated_velocity_poses_plotter.add_poses(
        "Graph VIO Integrated Velocity Poses", integrated_velocity_poses, linestyle="-"
    )
    integrated_velocity_poses_plotter.plot_positions(pdf)
    results_savers.save_rmse(
        integrated_velocity_poses,
        groundtruth_poses,
        results_csv_file,
        pdf,
        "Graph VIO Integrated Velocity Poses",
    )

    if len(imu_bias_extrapolated_poses) != 0:
        absolute_imu_bias_extrapolated_poses = (
            plot_conversions.absolute_poses_from_imu_bias_extrapolated_poses(
                imu_bias_extrapolated_poses, groundtruth_poses
            )
        )
        imu_bias_extrapolated_poses_plotter = plotters.MultiPosePlotter(
            "Time (s)",
            "Position (m)",
            "IMU Bias Extrapolated vs. Groundtruth Position",
            True,
        )
        imu_bias_extrapolated_poses_plotter.add_poses(
            "Groundtruth Poses",
            groundtruth_poses,
            linestyle="None",
            marker="o",
            markeredgewidth=0.1,
            markersize=1.5,
        )
        imu_bias_extrapolated_poses_plotter.add_poses(
            "IMU Bias Extrapolated Poses",
            absolute_imu_bias_extrapolated_poses,
            linestyle="-",
        )
        imu_bias_extrapolated_poses_plotter.plot(pdf)
        results_savers.save_rmse(
            imu_bias_extrapolated_poses,
            groundtruth_poses,
            results_csv_file,
            pdf,
            "IMU Bias Extrapolated Poses",
        )

    if len(depth_odometries) != 0:
        absolute_depth_odom_relative_poses = (
            plot_conversions.absolute_poses_from_relative_poses(
                plot_conversions.poses_from_depth_odometries(depth_odometries),
                groundtruth_poses,
            )
        )
        depth_odom_relative_poses_plotter = plotters.MultiPosePlotter(
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

        num_features_plotter = (
            plot_conversions.num_features_plotter_from_depth_odometries(
                depth_odometries
            )
        )
        num_features_plotter.plot(pdf)

        runtime_plotter = plot_conversions.runtime_plotter_from_depth_odometries(
            depth_odometries
        )
        runtime_plotter.plot(pdf)

    standstill_plotter = plot_conversions.standstill_plotter_from_states(
        graph_vio_states
    )
    standstill_plotter.plot(pdf)

    num_states_plotter = plot_conversions.num_states_plotter_from_states(
        graph_vio_states
    )
    num_states_plotter.plot(pdf)

    duration_plotter = plot_conversions.duration_plotter_from_states(graph_vio_states)
    duration_plotter.plot(pdf)

    optimization_time_plotter = plot_conversions.optimization_time_plotter_from_states(
        graph_vio_states
    )
    optimization_time_plotter.plot(pdf)

    update_time_plotter = plot_conversions.update_time_plotter_from_states(
        graph_vio_states
    )
    update_time_plotter.plot(pdf)

    optimization_iterations_plotter = (
        plot_conversions.optimization_iterations_plotter_from_states(graph_vio_states)
    )
    optimization_iterations_plotter.plot(pdf)


# Loads poses from the provided bagfile, generates plots, and saves results to a pdf and csv file.
# The csv file contains results in (TODO: define format).
# The RMSE rel start and end time define the time range for evaluating the RMSE, in case the bag starts or ends with little/uninteresting movement
# that shouldn't be included in the RMSE calculation.
# The groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def load_data_and_create_vio_plots(
    bagfile,
    output_pdf_file,
    results_csv_file="vio_results.csv",
    groundtruth_bagfile=None,
    rmse_rel_start_time=0,
    rmse_rel_end_time=-1,
):
    # Load bagfile with VIO results
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

    # Load graph VIO states
    graph_vio_states = []
    message_reader.load_graph_vio_states(
        graph_vio_states, "/graph_vio/state", bag, bag_start_time
    )

    # Load IMU bias extrapolated poses
    imu_bias_extrapolated_poses = []
    message_reader.load_poses(
        imu_bias_extrapolated_poses, "/imu_bias_extrapolator/pose", bag, bag_start_time
    )

    # Load Depth Odometries
    depth_odometries = message_reader.load_depth_odometries(
        "/loc/depth/odom", bag, bag_start_time
    )
    bag.close()

    with PdfPages(output_pdf_file) as pdf:
        plot_vio_results(
            pdf,
            results_csv_file,
            groundtruth_poses,
            graph_vio_states,
            imu_bias_extrapolated_poses,
            depth_odometries,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument(
        "--output-file", default="vio_output.pdf", help="Output pdf file."
    )
    parser.add_argument(
        "--results-csv-file",
        default="vio_results.csv",
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
    load_data_and_create_vio_plots(
        args.bagfile,
        args.output_file,
        args.results_csv_file,
        args.groundtruth_bagfile,
        args.rmse_rel_start_time,
        args.rmse_rel_end_time,
    )
