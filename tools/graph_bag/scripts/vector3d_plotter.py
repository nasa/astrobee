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

import poses
import vector3ds

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import numpy as np

def unwrap_in_degrees(angles):
  return np.rad2deg(np.unwrap(np.deg2rad(angles)))

class Vector3dPlotter():

  def __init__(self, xlabel, ylabel, title, individual_plots):
    self.xlabel = xlabel
    self.ylabel = ylabel
    self.title = title
    self.individual_plots = individual_plots
    self.y_vals_vec = []

  def add_pose_position(self,
                        pose,
                        colors=['r', 'b', 'g'],
                        linestyle='-',
                        linewidth=1,
                        marker=None,
                        markeredgewidth=None,
                        markersize=1):
    position_plotter = Vector3dYVals(pose.pose_type, pose.times, pose.positions.xs, pose.positions.ys,
                                     pose.positions.zs, ['Pos. (X)', 'Pos. (Y)', 'Pos. (Z)'], colors, linestyle,
                                     linewidth, marker, markeredgewidth, markersize)
    self.add_y_vals(position_plotter)

  def add_pose_orientation(self,
                           pose,
                           colors=['r', 'b', 'g'],
                           linestyle='-',
                           linewidth=1,
                           marker=None,
                           markeredgewidth=None,
                           markersize=1):
    orientation_plotter = Vector3dYVals(pose.pose_type, pose.times, unwrap_in_degrees(pose.orientations.yaws), unwrap_in_degrees(pose.orientations.rolls),
                                        unwrap_in_degrees(pose.orientations.pitches),
                                        ['Orientation (Yaw)', 'Orientation (Roll)', 'Orientation (Pitch)'], colors,
                                        linestyle, linewidth, marker, markeredgewidth, markersize)
    self.add_y_vals(orientation_plotter)

  def add_y_vals(self, y_vals):
    self.y_vals_vec.append(y_vals)

  def plot(self, pdf, individual_plots=True):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.full_plot()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()

    if individual_plots:
      self.plot_xs(pdf)
      self.plot_ys(pdf)
      self.plot_zs(pdf)

  def plot_xs(self, pdf):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.plot_x()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()

  def plot_ys(self, pdf):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.plot_y()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()

  def plot_zs(self, pdf):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.plot_z()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()


class Vector3dYVals():

  def __init__(self,
               name,
               x_axis_vals,
               x_vals,
               y_vals,
               z_vals,
               labels,
               colors=['r', 'b', 'g'],
               linestyle='-',
               linewidth=1,
               marker=None,
               markeredgewidth=None,
               markersize=1):
    self.x_vals = x_vals
    self.y_vals = y_vals
    self.z_vals = z_vals
    self.x_axis_vals = x_axis_vals
    self.name = name
    self.x_label = name + ' ' + labels[0]
    self.y_label = name + ' ' + labels[1]
    self.z_label = name + ' ' + labels[2]
    self.x_color = colors[0]
    self.y_color = colors[1]
    self.z_color = colors[2]
    self.linestyle = linestyle
    self.linewidth = linewidth
    self.marker = marker
    self.markeredgewidth = markeredgewidth
    self.markersize = markersize

  def full_plot(self):
    self.plot_x()
    self.plot_y()
    self.plot_z()

  def plot_x(self):
    plt.plot(self.x_axis_vals,
             self.x_vals,
             label=self.x_label,
             color=self.x_color,
             linestyle=self.linestyle,
             marker=self.marker,
             markeredgewidth=self.markeredgewidth,
             markersize=self.markersize)

  def plot_y(self):
    plt.plot(self.x_axis_vals,
             self.y_vals,
             label=self.y_label,
             color=self.y_color,
             linewidth=self.linewidth,
             linestyle=self.linestyle,
             marker=self.marker,
             markeredgewidth=self.markeredgewidth,
             markersize=self.markersize)

  def plot_z(self):
    plt.plot(self.x_axis_vals,
             self.z_vals,
             label=self.z_label,
             color=self.z_color,
             linestyle=self.linestyle,
             marker=self.marker,
             markeredgewidth=self.markeredgewidth,
             markersize=self.markersize)
