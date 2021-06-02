#!/usr/bin/env python
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
import os
import sys

import numpy as np

import rosbag


def get_average_opt_and_update_times(bagfile):
  with rosbag.Bag(bagfile, 'r') as bag:
    optimization_times = []
    update_times = []
    for _, msg, _ in bag.read_messages(['/graph_loc/state']):
      optimization_times.append(msg.optimization_time)
      update_times.append(msg.update_time)

    mean_optimization_time = np.mean(optimization_times)
    min_optimization_time = np.min(optimization_times)
    max_optimization_time = np.max(optimization_times)
    stddev_optimization_time = np.std(optimization_times)
    print('Mean optimization time: ' + str(mean_optimization_time))
    print('Min optimization time: ' + str(min_optimization_time))
    print('Max optimization time: ' + str(max_optimization_time))
    print('Stddev optimization time: ' + str(stddev_optimization_time))

    mean_update_time = np.mean(update_times)
    min_update_time = np.min(update_times)
    max_update_time = np.max(update_times)
    stddev_update_time = np.std(update_times)
    print('Mean update time: ' + str(mean_update_time))
    print('Min update time: ' + str(min_update_time))
    print('Max update time: ' + str(max_update_time))
    print('Stddev update time: ' + str(stddev_update_time))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile')
  args = parser.parse_args()
  if not os.path.isfile(args.bagfile):
    print('Bag file ' + args.bagfile + ' does not exist.')
    sys.exit()

  get_average_opt_and_update_times(args.bagfile)
