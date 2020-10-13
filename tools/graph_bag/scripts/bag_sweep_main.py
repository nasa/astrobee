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

import bag_sweep 

import argparse

import os
import sys

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('config_file')
  parser.add_argument('output_dir')
  args = parser.parse_args()
  if not os.path.isfile(args.config_file):
    print('Config file ' + args.config_file + ' does not exist.')
    sys.exit()
  if os.path.isdir(args.output_dir):
    print('Output directory ' + args.output_dir + ' already exists.')
    sys.exit()
  os.makedirs(args.output_dir)

  bag_sweep.bag_sweep(args.config_file, args.output_dir)
