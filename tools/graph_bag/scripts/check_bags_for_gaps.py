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

import rosbag

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile')
  parser.add_argument('-m', '--max-time-diff', default=0.5)
  args = parser.parse_args()
  if not os.path.isfile(args.bagfile):
    print('Bag file ' + args.bagfile + ' does not exist.')
    sys.exit()

  # '/hw/imu', '/loc/of/features', '/loc/ml/features', '/loc/ar/features', '/mgt/img_sampler/nav_cam/image_record'
  topics = ['/hw/imu']

  with rosbag.Bag(args.bagfile, 'r') as bag:
    last_time = 0.0
    gaps = 0
    num_time_diffs = 0
    avg_time_diff = 0.0
    for topic, msg, t in bag.read_messages(topics):
      time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1.0e-9
      time_diff = time - last_time
      if last_time != 0 and time_diff > args.max_time_diff:
        print('Imu Gap: time: ' + str(time) + ', last_time: ' + str(last_time) + ', diff: ' + str(time_diff))
        gaps += 1
      if (last_time != 0):
        num_time_diffs += 1
        # Compute moving average to avoid overflow
        avg_time_diff += (time_diff - avg_time_diff) / num_time_diffs
      last_time = time
    print('Found ' + str(gaps) + ' imu gaps.')
    print('Average time diff: ' + str(avg_time_diff))

  topics = ['/mgt/img_sampler/nav_cam/image_record']

  with rosbag.Bag(args.bagfile, 'r') as bag:
    last_time = 0.0
    gaps = 0
    num_time_diffs = 0
    avg_time_diff = 0.0
    for topic, msg, t in bag.read_messages(topics):
      time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1.0e-9
      time_diff = time - last_time
      if last_time != 0 and time_diff > args.max_time_diff:
        print('Image Gap: time: ' + str(time) + ', last_time: ' + str(last_time) + ', diff: ' + str(time_diff))
        gaps += 1
      if (last_time != 0):
        num_time_diffs += 1
        # Compute moving average to avoid overflow
        avg_time_diff += (time_diff - avg_time_diff) / num_time_diffs
      last_time = time
    print('Found ' + str(gaps) + ' image gaps.')
    print('Average time diff: ' + str(avg_time_diff))
