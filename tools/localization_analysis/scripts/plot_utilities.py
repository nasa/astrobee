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
Utility functions for generating pose/velocity/bias and other types of plots.
"""

matplotlib.use("pdf")
import csv
import math

import geometry_msgs
import plot_conversions
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import vector3d_plotter

# TODO: remove this file....

# Plot velocities over time
def plot_velocities_vs_time(
    pdf,
    timestamped_velocities,
    title_prefix = ''
):
    plt.figure()
    velocity_xyz_vectors = plot_conversions.velocity_xyz_vectors_from_velocities(timestamped_velocities)
    times = plot_conversions.times_from_timestamped_objects(timestamped_velocities)
    labels = ['X', 'Y', 'Z']
    velocity_plotter = vector3d_plotter.Vector3dPlotter("Vel. ", times, velocity_xyz_vectors[0], velocity_xyz_vectors[1], velocity_xyz_vectors[2], labels) 
    velocity_plotter.full_plot()
    plt.xlabel("Time (s)")
    plt.ylabel("Velocities")
    plt.title(title_prefix + "Velocities")
    plt.legend(prop={"size": 6})
    pdf.savefig()
    plt.close()

# Plot poses given a set of poses and groundtruth poses
def plot_poses_and_groundtruth_poses(pdf, timestamped_poses, timestamped_groundtruth_poses, title_prefix = ''):
    plot_pose_and_groundtruth_positions(pdf, timestamped_poses, timestamped_groundtruth_poses, title_prefix)
    plot_pose_and_groundtruth_orientations(pdf, timestamped_poses, timestamped_groundtruth_poses, title_prefix)


# Plot positions given a set of poses and groundtruth poses
def plot_pose_and_groundtruth_positions(pdf, timestamped_poses, timestamped_groundtruth_poses, title_prefix = ''):
    position_plotter = multipose_plotter.MultiPosePlotter(
        "Time (s)",
        "Position (m)",
        title_prefix + "vs. Groundtruth Position",
        True,
    )
    position_plotter.add_pose_position(
        timestamped_groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
        position_plotter.add_pose_position(timestamped_poses)
        position_plotter.plot(pdf)


# Plot orientations given a set of poses and groundtruth poses
def plot_pose_and_groundtruth_orientations(pdf, poses, groundtruth_poses, title_prefix = ''):
    orientation_plotter = multipose_plotter.MultiPosePlotter(
        "Time (s)",
        "Orientation (deg)",
        title_prefix + "vs. Groundtruth Orientation",
        True,
    )
    orientation_plotter.add_pose_orientation(
        groundtruth_poses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    orientation_plotter.add_pose_orientation(poses)
    orientation_plotter.plot(pdf)

# Plot Covariances given a set of poses 
def plot_covariances(pdf, timestamped_poses, title_prefix = ''):
    plt.figure()
    title = title_prefix + "Covariance"
    diagonal_position_covariance_norms = plot_conversions.diagonal_position_covariance_norms_from_poses(timestamped_poses)
    times = plot_conversions.times_from_timestamped_objects(timestamped_poses)
    plt.plot(times, diagonal_position_covariance_norms, "r", linewidth=0.5, label=title)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(title)
    plt.legend(prop={"size": 6})
    pdf.savefig()
    plt.close()

# Plot feature counts given a set of feature counts and times.
def plot_feature_counts(
    feature_counts,
    times,
    label,
    color,
    linestyle="None",
    marker="o",
    markeredgewidth=0.1,
    markersize=1.5,
):
    plt.plot(
        times,
        feature_counts,
        color,
        linestyle=linestyle,
        marker=marker,
        markeredgewidth=markeredgewidth,
        markersize=markersize,
        label=label,
    )


# Plot optical flow features given a set of graph vio states
def plot_optical_flow_features(pdf, graph_vio_states):
    plt.figure()
    optical_flow_feature_counts = plot_conversions.optical_flow_feature_counts_from_graph_vio_states(graph_vio_states)
    times = plot_conversions.times_from_timestamped_objects(graph_vio_states)
    plot_feature_counts(
        optical_flow_feature_counts, 
        times,
        "Det. OF",
        "b",
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    optical_flow_factor_counts = plot_conversions.optical_flow_factor_counts_from_graph_vio_states(graph_vio_states)
    plot_feature_counts(
        optical_flow_factor_counts,
        times,
        "OF Factors",
        "r",
        marker="x",
        markeredgewidth=0.1,
        markersize=1.5,
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Optical Flow Feature Counts")
    plt.title("Optical Flow Feature Counts")
    plt.legend(prop={"size": 6})
    plt.ylim(ymin=-1)
    pdf.savefig()
    plt.close()
