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

import copy
import math
import numpy as np
import unittest

import ekf_graph 


class TestRMSESequence(unittest.TestCase):

  def test_prune_missing_timestamps_beginning_set(self):
    a_times = np.arange(10.0)
    a_xs = np.arange(10.0)
    a_ys = np.arange(10.0) + 1.0
    a_zs = np.arange(10.0) + 2.0
    b_times = np.arange(5.0)
    pruned_a = ekf_graph.prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times)
    self.assertEqual(len(pruned_a), len(b_times))
    self.assertTrue(np.allclose(pruned_a[:, 0], b_times, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 1], b_times + 1.0, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 2], b_times + 2.0, rtol=0))

  def test_prune_missing_timestamps_middle_set(self):
    a_times = np.arange(10.0)
    a_xs = np.arange(10.0)
    a_ys = np.arange(10.0) + 1.0
    a_zs = np.arange(10.0) + 2.0
    b_times = np.arange(3.0, 7.0)
    pruned_a = ekf_graph.prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times)
    self.assertEqual(len(pruned_a), len(b_times))
    self.assertTrue(np.allclose(pruned_a[:, 0], b_times, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 1], b_times + 1.0, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 2], b_times + 2.0, rtol=0))

  def test_prune_missing_timestamps_end_set(self):
    a_times = np.arange(10.0)
    a_xs = np.arange(10.0)
    a_ys = np.arange(10.0) + 1.0
    a_zs = np.arange(10.0) + 2.0
    b_times = np.arange(7.0, 10.0)
    pruned_a = ekf_graph.prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times)
    self.assertEqual(len(pruned_a), len(b_times))
    self.assertTrue(np.allclose(pruned_a[:, 0], b_times, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 1], b_times + 1.0, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 2], b_times + 2.0, rtol=0))

  def test_prune_missing_timestamps_scattered_set(self):
    a_times = np.arange(10.0)
    a_xs = np.arange(10.0)
    a_ys = np.arange(10.0) + 1.0
    a_zs = np.arange(10.0) + 2.0
    b_times = np.array([1.0, 5.0, 6.0, 9.0])
    pruned_a = ekf_graph.prune_missing_timestamps(a_xs, a_ys, a_zs, a_times, b_times)
    self.assertEqual(len(pruned_a), len(b_times))
    self.assertTrue(np.allclose(pruned_a[:, 0], b_times, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 1], b_times + 1.0, rtol=0))
    self.assertTrue(np.allclose(pruned_a[:, 2], b_times + 2.0, rtol=0))

  def test_rmse_matrix_ones(self):
    ones_mat = np.ones(shape=(5, 3))
    self.assertTrue(np.isclose(ekf_graph.rmse_matrix(ones_mat), math.sqrt(3.0), rtol=0))

  def test_rmse_matrix_1_2_2(self):
    col_0 = np.ones(5)
    col_1 = np.full(5,2.0)
    col_2 = np.full(5,2.0)
    mat = np.column_stack((col_0, col_1, col_2))
    self.assertTrue(np.isclose(ekf_graph.rmse_matrix(mat), 3.0, rtol=0))

if __name__ == '__main__':
  unittest.main()
