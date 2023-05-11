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
import poses
import vector3d_plotter

matplotlib.use("pdf")
import matplotlib.pyplot as plt
import numpy as np

# Convert a list of angles from radians to degrees.
def radians_to_degrees(angles):
    return np.rad2deg(np.unwrap(np.deg2rad(angles)))

# Plotter that allows plotting multiple pose positions and orientations in the same plots.
# Pass poses to plot using the add_pose_position(...) and add_pose_orientation(...) functions.
# Optionally also plot individual plots for each position or orientation axis (x/y/z or r/p/y) if individual plots is true.
class MultiPosePlotter:
    def __init__(self, xlabel, ylabel, title, individual_plots = False):
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title
        self.individual_plots = individual_plots
        self.vector3d_plotters = []

    # Add poses to position plots.
    def add_pose_position(
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
        position_plotter = vector3d_plotter.Vector3dPlotter(
            name,
            times,
            xyz_vectors[0],
            xyz_vectors[1],
            xyz_vectors[2],
            ["Pos. (X)", "Pos. (Y)", "Pos. (Z)"],
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )
        self.add_vector3d_plotter(position_plotter)

    # Add pose to orientation plots
    def add_pose_orientation(
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
        orientation_plotter = vector3d_plotter.Vector3dPlotter(
            name,
            times,
            radians_to_degrees(ypr_vectors[0]),
            radians_to_degrees(ypr_vectors[1]),
            radians_to_degrees(ypr_vectors[2]),
            ["Orientation (Yaw)", "Orientation (Roll)", "Orientation (Pitch)"],
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )
        self.add_vector3d_plotter(orientation_plotter)

    # Helper function to add vector 3d plotters that should be called during plotting.
    def add_vector3d_plotter(self, vector3d_plotter):
        self.vector3d_plotters.append(vector3d_plotter)

    # Plot each of the added y values. Optionally plot individual axes on seperate plots
    # if individual_plots set to True.
    def plot(self, pdf, individual_plots=True):
        plt.figure()
        for vector3d_plotter in self.vector3d_plotters:
            vector3d_plotter.full_plot()
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend(prop={"size": 6})
        pdf.savefig()
        plt.close()

        if individual_plots:
            self.plot_xs(pdf)
            self.plot_ys(pdf)
            self.plot_zs(pdf)

    # Plot x values. 
    def plot_xs(self, pdf):
        plt.figure()
        for vector3d_plotter in self.vector3d_plotters:
            vector3d_plotter.plot_x()
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend(prop={"size": 6})
        pdf.savefig()
        plt.close()

    # Plot y values. 
    def plot_ys(self, pdf):
        plt.figure()
        for vector3d_plotter in self.vector3d_plotters:
            vector3d_plotter.plot_y()
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend(prop={"size": 6})
        pdf.savefig()
        plt.close()

    # Plot z values. 
    def plot_zs(self, pdf):
        plt.figure()
        for vector3d_plotter in self.vector3d_plotters:
            vector3d_plotter.plot_z()
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend(prop={"size": 6})
        pdf.savefig()
        plt.close()
