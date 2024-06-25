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

import matplotlib

import plot_conversions

matplotlib.use("pdf")
import sys

import matplotlib.pyplot as plt
import numpy as np


# Class for plotting 3d vector values on the same and separate plots.
class Vector3dPlotter(object):
    def __init__(
        self,
        name,
        x_axis_vals,
        x_vals,
        y_vals,
        z_vals,
        labels,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        self.x_vals = x_vals
        self.y_vals = y_vals
        self.z_vals = z_vals
        self.x_axis_vals = x_axis_vals
        self.name = name
        self.x_label = name + " " + labels[0]
        self.y_label = name + " " + labels[1]
        self.z_label = name + " " + labels[2]
        self.x_color = colors[0]
        self.y_color = colors[1]
        self.z_color = colors[2]
        self.linestyle = linestyle
        self.linewidth = linewidth
        self.marker = marker
        self.markeredgewidth = markeredgewidth
        self.markersize = markersize

    # Plot x, y, and z values on the same plot.
    def plot_xyz(self):
        self.plot_x()
        self.plot_y()
        self.plot_z()

    # Plot x values to the plot.
    def plot_x(self):
        plt.plot(
            self.x_axis_vals,
            self.x_vals,
            label=self.x_label,
            color=self.x_color,
            linestyle=self.linestyle,
            marker=self.marker,
            markeredgewidth=self.markeredgewidth,
            markersize=self.markersize,
        )

    # Plot y values to the plot.
    def plot_y(self):
        plt.plot(
            self.x_axis_vals,
            self.y_vals,
            label=self.y_label,
            color=self.y_color,
            linewidth=self.linewidth,
            linestyle=self.linestyle,
            marker=self.marker,
            markeredgewidth=self.markeredgewidth,
            markersize=self.markersize,
        )

    # Plot z values to the plot.
    def plot_z(self):
        plt.plot(
            self.x_axis_vals,
            self.z_vals,
            label=self.z_label,
            color=self.z_color,
            linestyle=self.linestyle,
            marker=self.marker,
            markeredgewidth=self.markeredgewidth,
            markersize=self.markersize,
        )


# Plotter that allows plotting multiple vector3d values in the same plots.
# Pass vector3d values to plot using the add function.
# Optionally also plot individual plots for each vector component (x/y/z) if individual plots is true.
class MultiVector3dPlotter:
    def __init__(self, xlabel, ylabel, title, individual_plots=False):
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title
        self.individual_plots = individual_plots
        self.vector3d_plotters = []

    # Adds vector3d plotter to be plot
    def add(self, vector3d_plotter):
        self.vector3d_plotters.append(vector3d_plotter)

    # Plot each of the vector3d values. Optionally plot individual axes on seperate plots
    # if individual_plots set to True.
    def plot(self, pdf):
        plt.figure()
        for vector3d_plotter in self.vector3d_plotters:
            vector3d_plotter.plot_xyz()
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend(prop={"size": 6})
        pdf.savefig()
        plt.close()

        if self.individual_plots:
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


# Clip rotation angles so they stay between 0 and 360
def unwrap_in_degrees(angles):
    return np.rad2deg(np.unwrap(np.deg2rad(angles)))


# Class for plotting YPR orientation values on the same and separate plots.
class OrientationPlotter(Vector3dPlotter):
    def __init__(
        self,
        name,
        times,
        y_vals,
        p_vals,
        r_vals,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        super(OrientationPlotter, self).__init__(
            name,
            times,
            unwrap_in_degrees(y_vals),
            unwrap_in_degrees(p_vals),
            unwrap_in_degrees(r_vals),
            ["Orientation (Yaw)", "Orientation (Roll)", "Orientation (Pitch)"],
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )


# Class for plotting XYZ position values on the same and separate plots.
class PositionPlotter(Vector3dPlotter):
    def __init__(
        self,
        name,
        times,
        x_vals,
        y_vals,
        z_vals,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        super(PositionPlotter, self).__init__(
            name,
            times,
            x_vals,
            y_vals,
            z_vals,
            ["Position (X)", "Position (Y)", "Position (Z)"],
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )


# Class for plotting values.
class ValuePlotter(object):
    def __init__(
        self,
        title,
        x_axis_vals,
        y_axis_vals,
        xlabel,
        ylabel,
        label="",
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        self.title = title
        self.x_axis_vals = x_axis_vals
        self.y_axis_vals = y_axis_vals
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.label = label
        self.linestyle = linestyle
        self.linewidth = linewidth
        self.marker = marker
        self.markeredgewidth = markeredgewidth
        self.markersize = markersize

    # Plot values
    def plot(self, pdf):
        plt.figure()
        plt.plot(
            self.x_axis_vals,
            self.y_axis_vals,
            label=self.label,
            linestyle=self.linestyle,
            marker=self.marker,
            markeredgewidth=self.markeredgewidth,
            markersize=self.markersize,
        )
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend(prop={"size": 6})
        pdf.savefig()
        plt.close()


# Convert a list of angles from radians to degrees.
def radians_to_degrees(angles):
    return np.rad2deg(np.unwrap(np.deg2rad(angles)))


# Plotter that allows plotting multiple pose positions and orientations in the same plots.
# Pass poses to plot using the add_poses function.
# Optionally also plot individual plots for each position or orientation axis (x/y/z or r/p/y) if individual plots is true.
class MultiPosePlotter:
    def __init__(self, xlabel, ylabel, title, individual_plots=True):
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title
        self.orientations_plotter = MultiVector3dPlotter(
            xlabel, ylabel, title, individual_plots
        )
        self.positions_plotter = MultiVector3dPlotter(
            xlabel, ylabel, title, individual_plots
        )

    # Add poses to position and orientation plots
    def add_poses(
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
        self.add_pose_positions(
            name,
            timestamped_poses,
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )

        self.add_pose_orientations(
            name,
            timestamped_poses,
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )

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


# Plot RMSEs vs. bag file for a set of bag files
class BagRMSEsPlotter:
    def __init__(self, dataframe):
        dataframe.sort_values(by=["Bag"], inplace=True)
        self.dataframe = dataframe
        # Get bag name abbreviations
        bag_names = dataframe["Bag"].tolist()
        max_name_length = 45
        shortened_bag_names = [
            (
                bag_name[-1 * max_name_length :]
                if len(bag_name) > max_name_length
                else bag_name
            )
            for bag_name in bag_names
        ]
        self.bag_names = shortened_bag_names
        self.bag_name_indices = list(range(len(shortened_bag_names)))

    def plot(
        self,
        pdf,
        rmse_name,
    ):
        rmses = self.dataframe[rmse_name]
        plt.figure()
        plt.plot(
            self.bag_name_indices,
            rmses,
            "b",
            linestyle="None",
            marker="o",
            markeredgewidth=0.1,
            markersize=10.5,
        )
        plt.xticks(self.bag_name_indices, self.bag_names, fontsize=7, rotation=20)
        plt.ylabel(rmse_name + " RMSE")
        plt.title(rmse_name + " RMSE vs. Bag")
        x_range = (
            self.bag_name_indices[len(self.bag_name_indices) - 1]
            - self.bag_name_indices[0]
        )
        x_buffer = x_range * 0.1
        # Extend x axis on either side to make data more visible
        plt.xlim(
            [
                self.bag_name_indices[0] - x_buffer,
                self.bag_name_indices[len(self.bag_name_indices) - 1] + x_buffer,
            ]
        )
        plt.tight_layout()
        pdf.savefig()
        plt.close()
