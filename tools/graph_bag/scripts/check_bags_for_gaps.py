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
import utilities

import argparse
import os
import sys

import rosbag

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile')
  parser.add_argument('topic')
  parser.add_argument('-m', '--max-time-diff', default=0.5)
  args = parser.parse_args()
  if not os.path.isfile(args.bagfile):
    print('Bag file ' + args.bagfile + ' does not exist.')
    sys.exit()

  utilities.get_topic_rates(args.bagfile, args.topic, args.max_time_diff, True)
