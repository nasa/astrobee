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

import argparse

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import scipy.spatial.transform

import math
import rosbag
import geometry_msgs


class Poses:

  def __init__(self, pose_type, topic):
    self.xs = []
    self.ys = []
    self.zs = []
    self.times = []
    self.rolls = []
    self.pitches = []
    self.yaws = []
    self.pose_type = pose_type
    self.topic = topic


def add_pose(pose_msg, timestamp, poses):
  poses.xs.append(pose_msg.position.x)
  poses.ys.append(pose_msg.position.y)
  poses.zs.append(pose_msg.position.z)
  euler_angles = scipy.spatial.transform.Rotation.from_quat(
    [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z,
     pose_msg.orientation.w]).as_euler('ZYX', degrees=True)
  poses.yaws.append(euler_angles[0])
  poses.pitches.append(euler_angles[1])
  poses.rolls.append(euler_angles[2])
  poses.times.append(timestamp.secs + 1e-9 * timestamp.nsecs)


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


def plot_positions(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  labels = [poses.pose_type + ' Pos. (X)', poses.pose_type + ' Pos. (Y)', poses.pose_type + 'Pos. (Z)']
  plot_vals(poses.times, [poses.xs, poses.ys, poses.zs], labels, colors, linewidth, linestyle, marker, markeredgewidth,
            markersize)


def plot_orientations(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  labels = [
    poses.pose_type + ' Orientation (Yaw)', poses.pose_type + ' Orientation (Roll)',
    poses.pose_type + 'Orienation (Pitch)'
  ]
  plot_vals(poses.times, [poses.yaws, poses.rolls, poses.pitches], labels, colors, linewidth, linestyle, marker,
            markeredgewidth, markersize)


def add_pose_plots(pdf, sparse_mapping_poses, graph_localization_poses, imu_augmented_graph_localization_poses):
  colors = ['r', 'b', 'g']
  plt.figure()
  plot_positions(sparse_mapping_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_positions(graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_orientations(sparse_mapping_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_orientations(graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Imu Augmented Loc vs. Loc
  plt.figure()
  plot_positions(graph_localization_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_positions(imu_augmented_graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_orientations(graph_localization_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_orientations(imu_augmented_graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def load_msgs(vec_of_poses, bag):
  topics = [poses.topic for poses in vec_of_poses]
  for topic, msg, t in bag.read_messages(topics):
    for poses in vec_of_poses:
      if poses.topic == topic:
        add_pose(msg.pose, msg.header.stamp, poses)
        break
  bag.close()


def create_plots(bagfile, output_file):
  bag = rosbag.Bag(bagfile)
  sparse_mapping_poses = Poses('Sparse Mapping', 'sparse_mapping_pose')
  graph_localization_poses = Poses('Graph Localization', 'graph_loc/state')
  imu_augmented_graph_localization_poses = Poses('Imu Augmented Graph Localization', 'gnc/ekf')
  vec_of_poses = [sparse_mapping_poses, graph_localization_poses, imu_augmented_graph_localization_poses]
  load_msgs(vec_of_poses, bag)

  #TODO(rsoussan): add this as commandn line arg
  with PdfPages(output_file) as pdf:
    add_pose_plots(pdf, sparse_mapping_poses, graph_localization_poses, imu_augmented_graph_localization_poses)
