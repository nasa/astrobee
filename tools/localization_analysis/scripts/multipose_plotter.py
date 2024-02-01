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

import matplotlib
from orientation_plotter import OrientationPlotter
from position_plotter import PositionPlotter
import plot_conversions
import pose
from multivector3d_plotter import MultiVector3dPlotter

matplotlib.use("pdf")
import matplotlib.pyplot as plt
import numpy as np
import sys

# Convert a list of angles from radians to degrees.
def radians_to_degrees(angles):
    return np.rad2deg(np.unwrap(np.deg2rad(angles)))

# Plotter that allows plotting multiple pose positions and orientations in the same plots.
# Pass poses to plot using the add_poses function.
# Optionally also plot individual plots for each position or orientation axis (x/y/z or r/p/y) if individual plots is true.
class MultiPosePlotter:
    def __init__(self, xlabel, ylabel, title, individual_plots = True):
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title
        self.orientations_plotter = MultiVector3dPlotter(xlabel, ylabel, title, individual_plots)
        self.positions_plotter = MultiVector3dPlotter(xlabel, ylabel, title, individual_plots)

    # Add poses to position and orientation plots
    def add_poses(self,
        name, 
        timestamped_poses,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        self.add_pose_positions(name, timestamped_poses,
        colors,
        linestyle,
        linewidth,
        marker,
        markeredgewidth,
        markersize)

        self.add_pose_orientations(name, timestamped_poses,
        colors,
        linestyle,
        linewidth,
        marker,
        markeredgewidth,
        markersize)

 

    # Add poses to position plots.
    def add_pose_positions(
        self,
        name, 
        timestamped_poses,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):

        xyz_vectors = plot_conversions.xyz_vectors_from_poses(timestamped_poses)
        times = plot_conversions.times_from_timestamped_objects(timestamped_poses)
        position_plotter = PositionPlotter(
            name,
            times,
            xyz_vectors[0],
            xyz_vectors[1],
            xyz_vectors[2],
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )
        self.positions_plotter.add(position_plotter)

    # Add poses to orientation plots
    def add_pose_orientations(
        self,
        name, 
        timestamped_poses,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        ypr_vectors = plot_conversions.ypr_vectors_from_poses(timestamped_poses)
        times = plot_conversions.times_from_timestamped_objects(timestamped_poses)
        orientation_plotter = OrientationPlotter(
            name,
            times,
            radians_to_degrees(ypr_vectors[0]),
            radians_to_degrees(ypr_vectors[1]),
            radians_to_degrees(ypr_vectors[2]),
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )
        self.orientations_plotter.add(orientation_plotter)

    # Plot each of the added pose position values. Optionally plot individual axes on seperate plots
    # if individual_plots set to True.
    def plot_positions(self, pdf):
        self.positions_plotter.plot(pdf)

    # Plot each of the added pose orientation values. Optionally plot individual axes on seperate plots
    # if individual_plots set to True.
    def plot_orientations(self, pdf):
        self.orientations_plotter.plot(pdf)

    # Plot each of the added pose values. Plots positions and orientations in different plots. Optionally plot individual axes on seperate plots
    # if individual_plots set to True.
    def plot(self, pdf):
        self.plot_positions(pdf)
        self.plot_orientations(pdf)
