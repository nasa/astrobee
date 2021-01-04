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
import parameter_sweep
import utilities

import argparse

import os
import sys

def bag_and_parameter_sweep(graph_bag_params_list, output_dir):
  for graph_bag_params in graph_bag_params_list:
    # Save parameter sweep output in different directory for each bagfile, name directory using bagfile
    bag_name_prefix = os.path.splitext(os.path.basename(graph_bag_params.bagfile))[0]
    bag_output_dir = os.path.join(output_dir, bag_name_prefix)
    parameter_sweep.make_values_and_parameter_sweep(bag_output_dir, graph_bag_params.bagfile, graph_bag_params.map_file, graph_bag_params.image_topic, graph_bag_params.config_path, graph_bag_params.robot_config_file, graph_bag_params.world, graph_bag_params.use_image_features)

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
  output_dir = utilities.create_directory(args.output_dir)

  graph_bag_params_list = bag_sweep.load_params(args.config_file)
  bag_and_parameter_sweep(graph_bag_params_list, output_dir)

