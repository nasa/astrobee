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

import numpy as np

import math

# Assumes poses_a and poses_b are sorted in time
# TODO(rsoussan): Add copying of orientations
def get_same_timestamp_poses(poses_a, poses_b):
  trimmed_poses_a = poses.Poses(poses_a.pose_type, poses_a.topic)
  trimmed_poses_b = poses.Poses(poses_b.pose_type, poses_b.topic)
  poses_a_size = len(poses_a.times)
  poses_b_size = len(poses_b.times)
  a_index = 0
  b_index = 0
  # Increment a and b index as needed.  Add same timestamped poses to trimmed poses containers
  while (a_index < poses_a_size) and (b_index < poses_b_size):
    a_time = poses_a.times[a_index]
    b_time = poses_b.times[b_index] 
 
    if (a_time == b_time):
      trimmed_poses_a.positions.add_vector3d(poses_a.positions.get_vector3d(a_index))
      # trimmed_poses_a.orientations.add_vector3d(poses_a.orientations.get_vector3d(a_index))
      trimmed_poses_a.times.append(poses_a.times[a_index])
      trimmed_poses_b.positions.add_vector3d(poses_b.positions.get_vector3d(b_index))
      # trimmed_poses_b.orientations.add_vector3d(poses_b.orientations.get_vector3d(b_index))
      trimmed_poses_b.times.append(poses_b.times[b_index])
      a_index += 1
      b_index += 1
    elif (a_time < b_time):
      a_index += 1
    else:
      b_index += 1
  return trimmed_poses_a, trimmed_poses_b
      

# RMSE between two sequences of poses (position only). Only uses poses with the same timestamp
def rmse_timestamped_poses(poses_a, poses_b):
  trimmed_poses_a, trimmed_poses_b = get_same_timestamp_poses(poses_a, poses_b)
  assert len(trimmed_poses_a.times) == len(trimmed_poses_b.times), 'Length mismatch of poses'
  num_poses = len(trimmed_poses_a.times)
  mean_squared_error = 0
  for index in range(num_poses):
    a_vec = trimmed_poses_a.positions.get_numpy_vector(index) 
    b_vec = trimmed_poses_b.positions.get_numpy_vector(index) 
    difference_vec = np.subtract(a_vec, b_vec)
    squared_error = np.inner(difference_vec, difference_vec) 
    # Use rolling mean to avoid overflow
    mean_squared_error += (squared_error - mean_squared_error)/(index + 1)
  rmse = math.sqrt(mean_squared_error)
  return rmse
    
  
