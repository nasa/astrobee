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


class Vector3ds:

  def __init__(self):
    self.xs = []
    self.ys = []
    self.zs = []

  def add(self, x, y, z):
    self.xs.append(x)
    self.ys.append(y)
    self.zs.append(z)

  def add_vector3d(self, vector3d):
    self.xs.append(vector3d.x)
    self.ys.append(vector3d.y)
    self.zs.append(vector3d.z)


class Orientations:

  def __init__(self):
    self.yaws = []
    self.pitches = []
    self.rolls = []

  def add(self, yaw, pitch, roll):
    self.yaws.append(yaw)
    self.pitches.append(pitch)
    self.rolls.append(roll)


class Poses(object):

  def __init__(self, pose_type, topic):
    self.positions = Vector3ds()
    self.orientations = Orientations()
    self.times = []
    self.pose_type = pose_type
    self.topic = topic

  def add_pose(self, pose_msg, timestamp):
    self.positions.add(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)
    euler_angles = scipy.spatial.transform.Rotation.from_quat(
      [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z,
       pose_msg.orientation.w]).as_euler('ZYX', degrees=True)
    self.orientations.add(euler_angles[0], euler_angles[1], euler_angles[2])
    self.times.append(timestamp.secs + 1e-9 * timestamp.nsecs)


class LocStates(Poses):

  def __init__(self, loc_type, topic):
    super(LocStates, self).__init__(loc_type, topic)
    self.of_counts = []
    self.vl_counts = []
    self.accelerations = Vector3ds()
    self.velocities = Vector3ds()
    self.angular_velocities = Vector3ds()
    self.accelerometer_biases = Vector3ds()
    self.gyro_biases = Vector3ds()

  def add_loc_state(self, msg):
    self.add_pose(msg.pose, msg.header.stamp)
    self.of_counts.append(msg.of_count)
    self.vl_counts.append(msg.ml_count)
    self.accelerations.add_vector3d(msg.accel)
    self.velocities.add_vector3d(msg.velocity)
    self.angular_velocities.add_vector3d(msg.omega)
    self.accelerometer_biases.add_vector3d(msg.accel_bias)
    self.gyro_biases.add_vector3d(msg.gyro_bias)


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
  plot_vals(poses.times, [poses.positions.xs, poses.positions.ys, poses.positions.zs], labels, colors, linewidth,
            linestyle, marker, markeredgewidth, markersize)


def plot_orientations(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  labels = [
    poses.pose_type + ' Orientation (Yaw)', poses.pose_type + ' Orientation (Roll)',
    poses.pose_type + 'Orienation (Pitch)'
  ]
  plot_vals(poses.times, [poses.orientations.yaws, poses.orientations.rolls, poses.orientations.pitches], labels,
            colors, linewidth, linestyle, marker, markeredgewidth, markersize)


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


def plot_features(feature_counts, times, label, color, marker, markeredgewidth=0.1, markersize=1.5):
  plt.plot(times,
           feature_counts,
           color,
           marker=marker,
           markeredgewidth=markeredgewidth,
           markersize=markersize,
           label=label)


def add_feature_count_plots(pdf, graph_localization_states):
  plt.figure()
  plot_features(graph_localization_states.of_counts,
                graph_localization_states.times,
                'OF',
                'r',
                marker='x',
                markeredgewidth=0.1,
                markersize=1.5)
  plot_features(graph_localization_states.vl_counts,
                graph_localization_states.times,
                'VL',
                'b',
                marker='o',
                markeredgewidth=0.1,
                markersize=1.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Feature Counts (num features in graph)')
  plt.title('Feature Counts')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def add_other_loc_plots(pdf, graph_localization_states, imu_augmented_graph_localization_states):
  add_feature_count_plots(pdf, graph_localization_states)


def load_pose_msgs(vec_of_poses, bag):
  topics = [poses.topic for poses in vec_of_poses]
  for topic, msg, t in bag.read_messages(topics):
    for poses in vec_of_poses:
      if poses.topic == topic:
        poses.add_pose(msg.pose, msg.header.stamp)
        break


def load_loc_state_msgs(vec_of_loc_states, bag):
  topics = [loc_states.topic for loc_states in vec_of_loc_states]
  for topic, msg, t in bag.read_messages(topics):
    for loc_states in vec_of_loc_states:
      if loc_states.topic == topic:
        loc_states.add_loc_state(msg)
        break


def create_plots(bagfile, output_file):
  bag = rosbag.Bag(bagfile)
  sparse_mapping_poses = Poses('Sparse Mapping', 'sparse_mapping_pose')
  vec_of_poses = [sparse_mapping_poses]
  load_pose_msgs(vec_of_poses, bag)

  graph_localization_states = LocStates('Graph Localization', 'graph_loc/state')
  imu_augmented_graph_localization_states = LocStates('Imu Augmented Graph Localization', 'gnc/ekf')
  vec_of_loc_states = [graph_localization_states, imu_augmented_graph_localization_states]
  load_loc_state_msgs(vec_of_loc_states, bag)

  bag.close()

  with PdfPages(output_file) as pdf:
    add_pose_plots(pdf, sparse_mapping_poses, graph_localization_states, imu_augmented_graph_localization_states)
    add_other_loc_plots(pdf, graph_localization_states, imu_augmented_graph_localization_states)
