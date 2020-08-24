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
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import scipy.spatial.transform

import math
import rosbag
import geometry_msgs


class Poses:

  def __init__(self):
    self.xs = []
    self.ys = []
    self.zs = []
    self.times = []
    self.rolls = []
    self.pitches = []
    self.yaws = []


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


#TODO(rsoussan): add this as commandn line arg
bag = rosbag.Bag('/home/rsoussan/graph_bag_tests/results.bag')
sparse_mapping_poses = Poses()
graph_localization_poses = Poses()
imu_augmented_graph_localization_poses = Poses()
sparse_mapping_pose_topic = 'sparse_mapping_pose'
# graph_localization_pose_topic = 'graph_localization_pose'
graph_localization_loc_topic = 'graph_loc/state'
imu_augmentor_loc_topic = 'gnc/ekf'
for topic, msg, t in bag.read_messages(topics=[sparse_mapping_pose_topic, graph_localization_loc_topic]):
  if topic == sparse_mapping_pose_topic:
    add_pose(msg.pose.pose, msg.header.stamp, sparse_mapping_poses)
  elif topic == graph_localization_loc_topic:
    add_pose(msg.pose, msg.header.stamp, graph_localization_poses)
  elif topic == imu_augmentor_loc_topic:
    add_pose(msg.pose, msg.header.stamp, imu_augmented_graph_localization_poses)
bag.close()

colors = ['r', 'b', 'g']
#TODO(rsoussan): add this as commandn line arg
filename = 'output.pdf'
with PdfPages(filename) as pdf:
  # positions
  plt.figure()
  plt.plot(sparse_mapping_poses.times,
           sparse_mapping_poses.xs,
           colors[0],
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=1.5,
           label='Sparse Mapping Pos. (X)')
  plt.plot(sparse_mapping_poses.times,
           sparse_mapping_poses.ys,
           colors[1],
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=1.5,
           label='Sparse Mapping Pos. (Y)')
  plt.plot(sparse_mapping_poses.times,
           sparse_mapping_poses.zs,
           colors[2],
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=1.5,
           label='Sparse Mapping Pos. (Z)')

  plt.plot(graph_localization_poses.times,
           graph_localization_poses.xs,
           colors[0],
           linewidth=0.5,
           label='Graph Localization Pos. (X)')
  plt.plot(graph_localization_poses.times,
           graph_localization_poses.ys,
           colors[1],
           linewidth=0.5,
           label='Graph Localization Pos. (Y)')
  plt.plot(graph_localization_poses.times,
           graph_localization_poses.zs,
           colors[2],
           linewidth=0.5,
           label='Graph Localization Pos. (Z)')

  plt.plot(imu_augmented_graph_localization_poses.times,
           imu_augmented_graph_localization_poses.xs,
           colors[0],
           linewidth=0.5,
           linestyle='dashed',
           label='Imu Augmented Graph Localization Pos. (X)')
  plt.plot(imu_augmented_graph_localization_poses.times,
           imu_augmented_graph_localization_poses.ys,
           colors[1],
           linewidth=0.5,
           linestyle='dashed',
           label='Imu Augmented Graph Localization Pos. (Y)')
  plt.plot(imu_augmented_graph_localization_poses.times,
           imu_augmented_graph_localization_poses.zs,
           colors[2],
           linewidth=0.5,
           linestyle='dashed',
           label='Imu Augmented Graph Localization Pos. (Z)')

  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Position')
  plt.legend(prop={'size': 6})
  #plt.ylim(-max_y + 0.2, max_y + 0.2)
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plt.plot(sparse_mapping_poses.times,
           sparse_mapping_poses.yaws,
           colors[0],
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=1.5,
           label='Sparse Mapping Orientation (Yaw)')
  plt.plot(sparse_mapping_poses.times,
           sparse_mapping_poses.rolls,
           colors[1],
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=1.5,
           label='Sparse Mapping Orientation (Roll)')
  plt.plot(sparse_mapping_poses.times,
           sparse_mapping_poses.pitches,
           colors[2],
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=1.5,
           label='Sparse Mapping Orientation (Pitch)')

  plt.plot(graph_localization_poses.times,
           graph_localization_poses.yaws,
           colors[0],
           linewidth=0.5,
           label='Graph Localization Orientation (Yaw)')
  plt.plot(graph_localization_poses.times,
           graph_localization_poses.rolls,
           colors[1],
           linewidth=0.5,
           label='Graph Localization Orientation (Roll)')
  plt.plot(graph_localization_poses.times,
           graph_localization_poses.pitches,
           colors[2],
           linewidth=0.5,
           label='Graph Localization Orientation (Pitch')

  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Orientation')
  plt.legend(prop={'size': 6})
  #plt.ylim(-max_y + 0.2, max_y + 0.2)
  pdf.savefig()
  plt.close()
