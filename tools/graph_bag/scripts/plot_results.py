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

import loc_states
import ml_poses
import plot_helpers
import poses
import utilities

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import geometry_msgs
import math
import rosbag


def l2_map(vector3ds):
  return map(lambda (x, y, z): math.sqrt(x + y + z), zip(vector3ds.xs, vector3ds.ys, vector3ds.zs))


def add_pose_plots(pdf, sparse_mapping_poses, graph_localization_poses, imu_augmented_graph_localization_poses):
  colors = ['r', 'b', 'g']
  plt.figure()
  plot_helpers.plot_positions(sparse_mapping_poses,
                              colors,
                              linestyle='None',
                              marker='o',
                              markeredgewidth=0.1,
                              markersize=1.5)
  plot_helpers.plot_positions(graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Graph vs. Sparse Mapping Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_helpers.plot_orientations(sparse_mapping_poses,
                                 colors,
                                 linestyle='None',
                                 marker='o',
                                 markeredgewidth=0.1,
                                 markersize=1.5)
  plot_helpers.plot_orientations(graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Graph vs. Sparse Mapping Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Imu Augmented Loc vs. Loc
  plt.figure()
  plot_helpers.plot_positions(graph_localization_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_helpers.plot_positions(imu_augmented_graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Graph vs. Imu Augmented Graph Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_helpers.plot_orientations(graph_localization_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_helpers.plot_orientations(imu_augmented_graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Graph vs. Imu Augmented Graph Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def plot_features(feature_counts,
                  times,
                  label,
                  color,
                  linestyle='None',
                  marker='o',
                  markeredgewidth=0.1,
                  markersize=1.5):
  plt.plot(times,
           feature_counts,
           color,
           linestyle=linestyle,
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


def add_other_vector3d_plots(pdf, imu_augmented_graph_localization_states):
  # Acceleration
  plt.figure()
  plot_helpers.plot_vector3ds(imu_augmented_graph_localization_states.accelerations,
                              imu_augmented_graph_localization_states.times, 'Acc.')
  plt.xlabel('Time (s)')
  plt.ylabel('Acceleration (m/s^2)')
  plt.title('Acceleration')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Biases
  # Plot Accelerometer Biases on different pages since they can start with quite different
  # values, plotting on the same page will lead to a large y axis scale and hide subtle changes.
  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times,
           imu_augmented_graph_localization_states.accelerometer_biases.xs, 'r')
  plt.xlabel('Time (s)')
  plt.ylabel('Accelerometer Biases (X)')
  plt.title('Accelerometer Biases (X)')
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times,
           imu_augmented_graph_localization_states.accelerometer_biases.ys, 'r')
  plt.xlabel('Time (s)')
  plt.ylabel('Accelerometer Biases (Y)')
  plt.title('Accelerometer Biases (Y)')
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times,
           imu_augmented_graph_localization_states.accelerometer_biases.zs, 'r')
  plt.xlabel('Time (s)')
  plt.ylabel('Accelerometer Biases (Z)')
  plt.title('Accelerometer Biases (Z)')
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times, imu_augmented_graph_localization_states.gyro_biases.xs, 'r')
  plt.xlabel('Time (s)')
  plt.ylabel('Gyro Biases (X)')
  plt.title('Gyro Biases (X)')
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times, imu_augmented_graph_localization_states.gyro_biases.ys, 'r')
  plt.xlabel('Time (s)')
  plt.ylabel('Gyro Biases (Y)')
  plt.title('Gyro Biases (Y)')
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times, imu_augmented_graph_localization_states.gyro_biases.zs, 'r')
  plt.xlabel('Time (s)')
  plt.ylabel('Gyro Biases (Z)')
  plt.title('Gyro Biases (Z)')
  pdf.savefig()
  plt.close()

  # Angular Velocity
  plt.figure()
  plot_helpers.plot_vector3ds(imu_augmented_graph_localization_states.angular_velocities,
                              imu_augmented_graph_localization_states.times, 'Ang. Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Angular Velocities')
  plt.title('Angular Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Velocity
  plt.figure()
  plot_helpers.plot_vector3ds(imu_augmented_graph_localization_states.velocities,
                              imu_augmented_graph_localization_states.times, 'Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Velocities')
  plt.title('Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Position covariance
  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times,
           l2_map(imu_augmented_graph_localization_states.position_covariances),
           'r',
           linewidth=0.5,
           label='Position Covariance')
  plt.title('Position Covariance')
  plt.xlabel('Time (s)')
  plt.ylabel('Position Covariance')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Orientation covariance
  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times,
           l2_map(imu_augmented_graph_localization_states.orientation_covariances),
           'r',
           linewidth=0.5,
           label='Orientation Covariance')
  plt.title('Orientation Covariance (Quaternion)')
  plt.xlabel('Time (s)')
  plt.ylabel('Orientation Covariance')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Velocity covariance
  plt.figure()
  plt.plot(imu_augmented_graph_localization_states.times,
           l2_map(imu_augmented_graph_localization_states.velocity_covariances),
           'r',
           linewidth=0.5,
           label='Velocity Covariance')
  plt.title('Velocity Covariance')
  plt.xlabel('Time (s)')
  plt.ylabel('Velocity Covariance')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def plot_stats(pdf, graph_localization_states, sparse_mapping_poses):
  stats = ''
  rmse = utilities.rmse_timestamped_poses(graph_localization_states, sparse_mapping_poses)
  stats += 'rmse: ' + str(rmse)
  plt.figure()
  plt.axis('off')
  plt.text(0.0, 0.5, stats)
  pdf.savefig()


def add_imu_bias_tester_poses(pdf, imu_bias_tester_poses, sparse_mapping_poses):
  colors = ['r', 'b', 'g']
  plt.figure()
  plot_helpers.plot_positions(sparse_mapping_poses,
                              colors,
                              linestyle='None',
                              marker='o',
                              markeredgewidth=0.1,
                              markersize=1.5)
  plot_helpers.plot_positions(imu_bias_tester_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Imu Bias Tester vs. Sparse Mapping Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_helpers.plot_orientations(sparse_mapping_poses,
                                 colors,
                                 linestyle='None',
                                 marker='o',
                                 markeredgewidth=0.1,
                                 markersize=1.5)
  plot_helpers.plot_orientations(imu_bias_tester_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Imu Bias Tester vs. Sparse Mapping Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def add_other_loc_plots(pdf, graph_localization_states, imu_augmented_graph_localization_states):
  add_feature_count_plots(pdf, graph_localization_states)
  add_other_vector3d_plots(pdf, imu_augmented_graph_localization_states)


def load_pose_msgs(vec_of_poses, bag):
  topics = [poses.topic for poses in vec_of_poses]
  for topic, msg, t in bag.read_messages(topics):
    for poses in vec_of_poses:
      if poses.topic == topic:
        poses.add_msg(msg, msg.header.stamp)
        break


def load_loc_state_msgs(vec_of_loc_states, bag):
  topics = [loc_states.topic for loc_states in vec_of_loc_states]
  for topic, msg, t in bag.read_messages(topics):
    for loc_states in vec_of_loc_states:
      if loc_states.topic == topic:
        loc_states.add_loc_state(msg)
        break


def has_topic(bag, topic):
  topics = bag.get_type_and_topic_info().topics
  return topic in topics


def create_plots(bagfile, output_file, use_ml_features_poses=False):
  bag = rosbag.Bag(bagfile)

  has_imu_augmented_graph_localization_state = has_topic(bag, '/gnc/ekf')
  has_imu_bias_tester_poses = has_topic(bag, '/imu_bias_tester/pose')
  sparse_mapping_poses = poses.Poses('Sparse Mapping',
                                     '/sparse_mapping/pose') if not use_ml_features_poses else ml_poses.Poses(
                                       'Sparse Mapping', '/loc/ml/features')
  imu_bias_tester_poses = poses.Poses('Imu Bias Tester', '/imu_bias_tester/pose')
  vec_of_poses = [sparse_mapping_poses, imu_bias_tester_poses]
  load_pose_msgs(vec_of_poses, bag)

  graph_localization_states = loc_states.LocStates('Graph Localization', '/graph_loc/state')
  imu_augmented_graph_localization_states = loc_states.LocStates('Imu Augmented Graph Localization', '/gnc/ekf')
  vec_of_loc_states = [graph_localization_states, imu_augmented_graph_localization_states]
  load_loc_state_msgs(vec_of_loc_states, bag)

  bag.close()

  with PdfPages(output_file) as pdf:
    add_pose_plots(pdf, sparse_mapping_poses, graph_localization_states, imu_augmented_graph_localization_states)
    if has_imu_bias_tester_poses:
      add_imu_bias_tester_poses(pdf, imu_bias_tester_poses, sparse_mapping_poses)
    if has_imu_augmented_graph_localization_state:
      add_other_loc_plots(pdf, graph_localization_states, imu_augmented_graph_localization_states)
    else:
      add_other_loc_plots(pdf, graph_localization_states, graph_localization_states)
    plot_stats(pdf, graph_localization_states, sparse_mapping_poses)
