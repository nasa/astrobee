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
import rmse_utilities 

import math
import numpy as np
import unittest

def make_poses(times, xs, ys, zs):
  new_poses = poses.Poses('', '')
  new_poses.times = times
  new_poses.positions.xs = xs
  new_poses.positions.ys = ys
  new_poses.positions.zs = zs
  return new_poses

class TestRMSESequence(unittest.TestCase):

  def test_prune_missing_timestamps_beginning_set(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    b_times = np.arange(5.0)
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(b_times, xs, ys, zs) 
    
    trimmed_a, trimmed_b = rmse_utilities.get_same_timestamp_poses(poses_a, poses_b)
    self.assertEqual(len(trimmed_a.times), len(trimmed_b.times))
    self.assertEqual(len(trimmed_a.times), 5)
    self.assertTrue(np.allclose(trimmed_a.times, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.xs, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.ys, b_times + 1, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.zs, b_times + 2, rtol=0))

  def test_prune_missing_timestamps_middle_set(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    b_times = np.arange(3.0, 7.0)
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(b_times, xs, ys, zs) 
    
    trimmed_a, trimmed_b = rmse_utilities.get_same_timestamp_poses(poses_a, poses_b)
    self.assertEqual(len(trimmed_a.times), len(trimmed_b.times))
    self.assertEqual(len(trimmed_a.times), 4)
    self.assertTrue(np.allclose(trimmed_a.times, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.xs, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.ys, b_times + 1, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.zs, b_times + 2, rtol=0))


  def test_prune_missing_timestamps_end_set(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    b_times = np.arange(7.0, 10.0)
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(b_times, xs, ys, zs) 
    
    trimmed_a, trimmed_b = rmse_utilities.get_same_timestamp_poses(poses_a, poses_b)
    self.assertEqual(len(trimmed_a.times), len(trimmed_b.times))
    self.assertEqual(len(trimmed_a.times), 3)
    self.assertTrue(np.allclose(trimmed_a.times, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.xs, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.ys, b_times + 1, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.zs, b_times + 2, rtol=0))

  def test_prune_missing_timestamps_scattered_set(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    b_times = np.array([1.0, 5.0, 6.0, 9.0])
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(b_times, xs, ys, zs) 
    
    trimmed_a, trimmed_b = rmse_utilities.get_same_timestamp_poses(poses_a, poses_b)
    self.assertEqual(len(trimmed_a.times), len(trimmed_b.times))
    self.assertTrue(np.allclose(trimmed_a.times, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.xs, b_times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.ys, b_times + 1, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.zs, b_times + 2, rtol=0))

  def test_prune_missing_timestamps_disjoint_set(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    b_times = np.arange(11, 20)
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(b_times, xs, ys, zs) 
    
    trimmed_a, trimmed_b = rmse_utilities.get_same_timestamp_poses(poses_a, poses_b)
    self.assertEqual(len(trimmed_a.times), 0)
    self.assertEqual(len(trimmed_b.times), 0)


  def test_prune_missing_timestamps_some_overlap(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    b_times = np.arange(8.0, 20.0)
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(b_times, xs, ys, zs) 
   
    expected_time_range = np.arange(8.0, 10.0) 
    trimmed_a, trimmed_b = rmse_utilities.get_same_timestamp_poses(poses_a, poses_b)
    self.assertEqual(len(trimmed_a.times), len(trimmed_b.times))
    self.assertTrue(np.allclose(trimmed_a.times, trimmed_b.times, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.times, expected_time_range, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.xs, expected_time_range, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.ys, expected_time_range + 1, rtol=0))
    self.assertTrue(np.allclose(trimmed_a.positions.zs, expected_time_range + 2, rtol=0))

    
  def test_rmse_same_poses(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(a_times, xs, ys, zs) 
  
    rmse = rmse_utilities.rmse_timestamped_poses(poses_a, poses_b)
    self.assertTrue(np.isclose(rmse, 0, rtol=0))

  def test_rmse_off_by_one(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(a_times, xs + 1, ys, zs) 
  
    rmse = rmse_utilities.rmse_timestamped_poses(poses_a, poses_b)
    self.assertTrue(np.isclose(rmse, 1.0, rtol=0))

  def test_rmse_all_off_by_one(self):
    a_times = np.arange(10.0)
    xs = np.arange(10.0)
    ys = np.arange(10.0) + 1.0
    zs = np.arange(10.0) + 2.0
    poses_a = make_poses(a_times, xs, ys, zs) 
    poses_b = make_poses(a_times, xs + 1, ys + 1, zs + 1) 
  
    rmse = rmse_utilities.rmse_timestamped_poses(poses_a, poses_b)
    self.assertTrue(np.isclose(rmse, math.sqrt(3.0), rtol=0))
    
if __name__ == '__main__':
  unittest.main()
