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


def plot_vals(x_axis_vals,
              vec_of_y_axis_vals,
              labels,
              colors,
              linewidth=1,
              linestyle='-',
              marker=None,
              markeredgewidth=None,
              markersize=1):
  for index, _ in enumerate(vec_of_y_axis_vals):
    plt.plot(x_axis_vals,
             vec_of_y_axis_vals[index],
             colors[index],
             linestyle=linestyle,
             linewidth=linewidth,
             marker=marker,
             markeredgewidth=markeredgewidth,
             markersize=markersize,
             label=labels[index])


def plot_vector3ds(vector3ds,
                   times,
                   label,
                   colors=['r', 'g', 'b'],
                   linewidth=1,
                   linestyle='-',
                   marker=None,
                   markeredgewidth=None,
                   markersize=1):
  labels = [label + ' (X)', label + ' (Y)', label + ' (Z)']
  plot_vals(times, [vector3ds.xs, vector3ds.ys, vector3ds.zs], labels, colors, linewidth, linestyle, marker,
            markeredgewidth, markersize)


def plot_positions(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  plot_vector3ds(poses.positions,
                 poses.times,
                 poses.pose_type + ' Pos.',
                 linewidth=linewidth,
                 linestyle=linestyle,
                 marker=marker,
                 markeredgewidth=markeredgewidth,
                 markersize=markersize)


def plot_orientations(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  labels = [
    poses.pose_type + ' Orientation (Yaw)', poses.pose_type + ' Orientation (Roll)',
    poses.pose_type + ' Orienation (Pitch)'
  ]
  plot_vals(poses.times, [poses.orientations.yaws, poses.orientations.rolls, poses.orientations.pitches], labels,
            colors, linewidth, linestyle, marker, markeredgewidth, markersize)
