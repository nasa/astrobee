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
