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

import numpy as np

import math

# RMSE of matrix interpreted as n (dim(rows) vectors of dim(col) 
def rmse_matrix(x):
  return np.mean(np.sqrt(np.sum(np.square(x), axis=1)))

# Removes the ith entry of a_xs, a_ys, and a_zs if a_times[i] is 
# not in b_times[i].  Assumes timestamp vectors are sorted
def prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times):
  a_index = 0
  pruned_a_matrix = np.empty(shape=(len(b_times), 3)) 
  for b_index, b_time in enumerate(b_times):
    while not np.isclose(a_times[a_index], b_time) and a_times[a_index] < b_time and a_index < len(a_times):
      a_index += 1 
    pruned_a_matrix[b_index] = np.array([a_xs[a_index], a_ys[a_index], a_zs[a_index]])
  return pruned_a_matrix

# RMSE between two sequences of timestamped positions. Prunes timestamped positions in sequence a
# not present in sequence b. 
def rmse_timestamped_sequences(a_xs, a_ys, a_zs, a_times, b_xs, b_ys, b_zs, b_times):
  a_positions = prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times)
  b_positions = np.column_stack((b_xs, b_ys, b_zs))
  return rmse_matrix(a_positions - b_positions)

def rmse_timestamped_poses(poses_a, poses_b):
  return rmse_timestamped_sequences(poses_a.positions.xs, poses_a.positions.ys, poses_a.positions.zs, poses_a.times, poses_b.positions.xs, poses_b.positions.ys, poses_b.positions.zs, poses_b.times)
