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
Creates plots for VIO results. Adds plots for poses, velocities, IMU bias values, 
integrated velocities forming poses, and detectected
features counts. Optionally plots groundtruth poses along with other pose data if groundtruth is provided.
"""

import argparse
import os
import sys

import message_reader 
from multipose_plotter import MultiPosePlotter
from multivector3d_plotter import MultiVector3dPlotter
import plot_conversions
from timestamped_pose import TimestampedPose
#import plotting_utilities

import matplotlib
matplotlib.use("pdf")
import csv
import math

import geometry_msgs
import matplotlib.pyplot as plt
import rosbag
from matplotlib.backends.backend_pdf import PdfPages

def plot_vio_results(
    pdf,
    groundtruth_poses,
    graph_vio_states,
):
    poses_plotter = MultiPosePlotter("Time (s)", "Position (m)", "Graph vs. Groundtruth Position", True)
    poses_plotter.add_poses(
        "Groundtruth Poses", 
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )

    graph_vio_poses = plot_conversions.adjusted_graph_vio_poses_from_graph_vio_states(graph_vio_states, groundtruth_poses) 
    poses_plotter.add_poses(
        "Graph VIO Poses", 
        graph_vio_poses,
        linestyle="-",
    )

#    if ar_tag_poses.times:
#        position_plotter.add_pose_position(
#            ar_tag_poses,
#            linestyle="None",
#            marker="x",
#            markeredgewidth=0.1,
#            markersize=1.5,
#        )
    poses_plotter.plot(pdf)

    velocities_plotter = MultiVector3dPlotter("Time (s)", "Velocity (m/s)", "Graph VIO Velocities", True)
    graph_vio_velocity_plotter = plot_conversions.velocity_plotter_from_graph_vio_states(graph_vio_states) 
    velocities_plotter.add(graph_vio_velocity_plotter)
    velocities_plotter.plot(pdf)

    accel_bias_plotters = MultiVector3dPlotter("Time (s)", "Accelerometer Bias (m/s^2)", "Graph VIO Accel. Biases", True)
    graph_vio_accel_bias_plotter = plot_conversions.accel_bias_plotter_from_graph_vio_states(graph_vio_states) 
    accel_bias_plotters.add(graph_vio_accel_bias_plotter)
    accel_bias_plotters.plot(pdf)

    gyro_bias_plotters = MultiVector3dPlotter("Time (s)", "Gyro Bias (rad/s^2)", "Graph VIO Gyro. Biases", True)
    graph_vio_gyro_bias_plotter = plot_conversions.gyro_bias_plotter_from_graph_vio_states(graph_vio_states) 
    gyro_bias_plotters.add(graph_vio_gyro_bias_plotter)
    gyro_bias_plotters.plot(pdf)

    of_count_plotter = plot_conversions.optical_flow_feature_count_plotter_from_graph_vio_states(graph_vio_states)
    of_count_plotter.plot(pdf)

    of_num_factors_plotter = plot_conversions.optical_flow_factor_count_plotter_from_graph_vio_states(graph_vio_states)
    of_num_factors_plotter.plot(pdf)


    integrated_velocity_poses = plot_conversions.absolute_poses_from_integrated_graph_vio_state_velocities(graph_vio_states, groundtruth_poses)
    integrated_velocity_poses_plotter = MultiPosePlotter("Time (s)", "Position (m)", "Integrated Velocities vs. Groundtruth Position", True)
    integrated_velocity_poses_plotter.add_poses(
        "Groundtruth Poses", 
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    integrated_velocity_poses_plotter.add_poses(
        "Graph VIO Integrated Velocity Poses", 
        integrated_velocity_poses,
        linestyle="-",
    )
    integrated_velocity_poses_plotter.plot_positions(pdf)






#
#    # Imu Augmented Loc vs. Loc
#    position_plotter = vector3d_plotter.Vector3dPlotter(
#        "Time (s)", "Position (m)", "Graph vs. IMU Augmented Graph Position", True
#    )
#    position_plotter.add_pose_position(
#        graph_localization_states,
#        linestyle="None",
#        marker="o",
#        markeredgewidth=0.1,
#        markersize=1.5,
#    )
#
#    position_plotter.add_pose_position(
#        imu_augmented_graph_localization_poses, linewidth=0.5
#    )
#    position_plotter.plot(pdf)
#
#    # orientations
#    orientation_plotter = vector3d_plotter.Vector3dPlotter(
#        "Time (s)",
#        "Orientation (deg)",
#        "Graph vs. IMU Augmented Graph Orientation",
#        True,
#    )
#    orientation_plotter.add_pose_orientation(
#        graph_localization_states, marker="o", markeredgewidth=0.1, markersize=1.5
#    )
#    orientation_plotter.add_pose_orientation(
#        imu_augmented_graph_localization_poses, linewidth=0.5
#    )
#    orientation_plotter.plot(pdf)
#
#    # Velocity
#    # TODO: add plot_velocities function!!
#    plt.figure()
#    plot_helpers.plot_vector3ds(
#        graph_localization_states.velocities, graph_localization_states.times, "Vel."
#    )
#    plt.xlabel("Time (s)")
#    plt.ylabel("Velocities")
#    plt.title("Graph Velocities")
#    plt.legend(prop={"size": 6})
#    pdf.savefig()
#    plt.close()
#
#    # Integrated Velocities
#    integrated_graph_localization_states = utilities.integrate_velocities(
#        graph_localization_states
#    )
#    plot_positions(
#        pdf, integrated_graph_localization_states, groundtruth_poses, ar_tag_poses
#    )


# Loads poses from the provided bagfile, generates plots, and saves results to a pdf and csv file.
# The csv file contains results in (TODO: define format).
# The RMSE rel start and end time define the time range for evaluating the RMSE, in case the bag starts or ends with little/uninteresting movement 
# that shouldn't be included in the RMSE calculation.
# The groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def load_data_and_create_vio_plots(
    bagfile,
    output_pdf_file,
    output_csv_file="vio_results.csv",
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
    message_reader.load_poses(groundtruth_poses, "/sparse_mapping/pose", groundtruth_bag, bag_start_time)


    # Load graph VIO states 
    graph_vio_states = []
    message_reader.load_graph_vio_states(graph_vio_states, "/graph_vio/state", bag, bag_start_time)
    bag.close()

    with PdfPages(output_pdf_file) as pdf:
        plot_vio_results(
            pdf,
            groundtruth_poses,
            #ar_tag_poses,
            graph_vio_states,
            #imu_augmented_graph_localization_states,
        )
    #    add_other_loc_plots(
    #        pdf, graph_localization_states, graph_localization_states
    #    )
    #    plot_loc_state_stats(
    #    pdf,
    #    graph_localization_states,
    #    groundtruth_poses,
    #    output_csv_file,
    #    rmse_rel_start_time=rmse_rel_start_time,
    #    rmse_rel_end_time=rmse_rel_end_time,
    #)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument("--output-file", default="output.pdf", help="Output pdf file.")
    parser.add_argument(
        "--output-csv-file",
        default="results.csv",
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
    load_data_and_create_vio_plots(
        args.bagfile,
        args.output_file,
        args.output_csv_file,
        args.groundtruth_bagfile,
        args.rmse_rel_start_time,
        args.rmse_rel_end_time,
    )
