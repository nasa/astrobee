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
from vector3d_plotter import Vector3dPlotter
import plot_conversions
import pose
import vector3d_plotter

matplotlib.use("pdf")
import matplotlib.pyplot as plt
import numpy as np
import sys

# Plotter that allows plotting multiple vector3d values in the same plots.
# Pass vector3d values to plot using the add function.
# Optionally also plot individual plots for each vector component (x/y/z) if individual plots is true.
class MultiVector3dPlotter:
    def __init__(self, xlabel, ylabel, title, individual_plots = False):
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
