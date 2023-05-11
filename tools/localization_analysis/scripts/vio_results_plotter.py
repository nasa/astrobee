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

#import os
#import sys

import bag_utilities
import plotting_utilities

import matplotlib
matplotlib.use("pdf")
import csv
import math

import geometry_msgs
import matplotlib.pyplot as plt
import rosbag
from matplotlib.backends.backend_pdf import PdfPages

# TODO: rename this! plot_vio_results?
def add_graph_plots(
    pdf,
    groundtruth_poses,
    graph_localization_states,
):
    colors = ["r", "b", "g"]
    position_plotter = vector3d_plotter.Vector3dPlotter(
        "Time (s)", "Position (m)", "Graph vs. Groundtruth Position", True
    )
    position_plotter.add_pose_position(
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    if ar_tag_poses.times:
        position_plotter.add_pose_position(
            ar_tag_poses,
            linestyle="None",
            marker="x",
            markeredgewidth=0.1,
            markersize=1.5,
        )
    position_plotter.add_pose_position(graph_localization_states)
    position_plotter.plot(pdf)

    # orientations
    orientation_plotter = vector3d_plotter.Vector3dPlotter(
        "Time (s)", "Orientation (deg)", "Graph vs. Groundtruth Orientation", True
    )
    orientation_plotter.add_pose_orientation(
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    if ar_tag_poses.times:
        orientation_plotter.add_pose_orientation(
            ar_tag_poses,
            linestyle="None",
            marker="x",
            markeredgewidth=0.1,
            markersize=1.5,
        )
    orientation_plotter.add_pose_orientation(graph_localization_states)
    orientation_plotter.plot(pdf)

    # Imu Augmented Loc vs. Loc
    position_plotter = vector3d_plotter.Vector3dPlotter(
        "Time (s)", "Position (m)", "Graph vs. IMU Augmented Graph Position", True
    )
    position_plotter.add_pose_position(
        graph_localization_states,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )

    position_plotter.add_pose_position(
        imu_augmented_graph_localization_poses, linewidth=0.5
    )
    position_plotter.plot(pdf)

    # orientations
    orientation_plotter = vector3d_plotter.Vector3dPlotter(
        "Time (s)",
        "Orientation (deg)",
        "Graph vs. IMU Augmented Graph Orientation",
        True,
    )
    orientation_plotter.add_pose_orientation(
        graph_localization_states, marker="o", markeredgewidth=0.1, markersize=1.5
    )
    orientation_plotter.add_pose_orientation(
        imu_augmented_graph_localization_poses, linewidth=0.5
    )
    orientation_plotter.plot(pdf)

    # Velocity
    # TODO: add plot_velocities function!!
    plt.figure()
    plot_helpers.plot_vector3ds(
        graph_localization_states.velocities, graph_localization_states.times, "Vel."
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Velocities")
    plt.title("Graph Velocities")
    plt.legend(prop={"size": 6})
    pdf.savefig()
    plt.close()

    # Integrated Velocities
    integrated_graph_localization_states = utilities.integrate_velocities(
        graph_localization_states
    )
    plot_positions(
        pdf, integrated_graph_localization_states, groundtruth_poses, ar_tag_poses
    )


# Groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def create_plots(
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
    # Use sparse mapping poses as groundtruth. TODO: pass this as an option?
    groundtruth_poses = poses.Poses("Groundtruth", "/sparse_mapping/pose")
    groundtruth_vec_of_poses = [groundtruth_poses]
    load_pose_msgs(groundtruth_vec_of_poses, groundtruth_bag, bag_start_time)

    # Load VIO states
    graph_vio_states = vio_states.LocStates(
        "Graph Localization", "/graph_loc/state"
    )
    vec_of_vio_states = [
        graph_vio_states
    ]
    load_vio_state_msgs(vec_of_vio_states, bag, bag_start_time)
    bag.close()

    with PdfPages(output_pdf_file) as pdf:
        add_graph_plots(
            pdf,
            groundtruth_poses,
            ar_tag_poses,
            graph_vio_states,
            imu_augmented_graph_localization_states,
        )
        add_other_loc_plots(
            pdf, graph_localization_states, graph_localization_states
        )
        plot_loc_state_stats(
        pdf,
        graph_localization_states,
        groundtruth_poses,
        output_csv_file,
        rmse_rel_start_time=rmse_rel_start_time,
        rmse_rel_end_time=rmse_rel_end_time,
    )
