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
matplotlib.use("pdf")
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
