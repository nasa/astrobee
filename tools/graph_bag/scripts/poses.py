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

import vector3ds
import orientations
from rmse_utilities import get_same_timestamp_poses, position_squared_difference, orientation_squared_difference

import scipy.spatial.transform
import numpy as np


class Poses(object):

  def __init__(self, pose_type, topic):
    self.positions = vector3ds.Vector3ds()
    self.orientations = orientations.Orientations()
    self.times = []
    self.pose_type = pose_type
    self.topic = topic

  def add_pose(self, pose_msg, timestamp, bag_start_time=0):
    self.positions.add(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)
    euler_angles = scipy.spatial.transform.Rotation.from_quat(
      [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z,
       pose_msg.orientation.w]).as_euler('ZYX', degrees=True)
    self.orientations.add(euler_angles[0], euler_angles[1], euler_angles[2])
    self.times.append(timestamp.secs + 1e-9 * timestamp.nsecs - bag_start_time)

  def add_msg(self, msg, timestamp, bag_start_time=0):
    self.add_pose(msg.pose, timestamp, bag_start_time)

  def position_vector(self, index):
    return [self.positions.xs[index], self.positions.ys[index], self.positions.zs[index]]

  def compute_error(self, poses_b, abs_tol=0, rel_start_time=0, rel_end_time=-1):
    trimmed_poses_a, trimmed_poses_b = get_same_timestamp_poses(self, poses_b, True, abs_tol,
                                                                rel_start_time, rel_end_time)
    assert len(trimmed_poses_a.times) == len(trimmed_poses_b.times), 'Length mismatch of poses'
    num_poses = len(trimmed_poses_a.times)
    position_error_hist = np.array([])
    orientation_error_hist = np.array([])
    time_hist = np.array([])
    for index in range(num_poses):
      # Position Error
      a_vec = trimmed_poses_a.positions.get_numpy_vector(index)
      b_vec = trimmed_poses_b.positions.get_numpy_vector(index)
      position_squared_error = position_squared_difference(a_vec, b_vec)
      position_error_hist = np.hstack((position_error_hist, np.sqrt(position_squared_error)))
      # Orientation Error
      a_rot = trimmed_poses_a.orientations.get_rotation(index)
      b_rot = trimmed_poses_b.orientations.get_rotation(index)
      orientation_squared_error = orientation_squared_difference(a_rot, b_rot)
      orientation_error_hist = np.hstack((orientation_error_hist, np.sqrt(orientation_squared_error)))
      # Time
      time_hist = np.hstack((time_hist, trimmed_poses_a.times[index]))

    return position_error_hist, orientation_error_hist, time_hist
