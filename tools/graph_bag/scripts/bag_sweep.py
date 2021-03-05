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

import average_results
import multiprocessing_helpers

import csv
import itertools
import multiprocessing
import os
import plot_bag_sweep_results
import sys


class GraphBagParams(object):

  def __init__(self, bagfile, map_file, image_topic, config_path, robot_config_file, world, use_image_features,
               groundtruth_bagfile):
    self.bagfile = bagfile
    self.map_file = map_file
    self.image_topic = image_topic
    self.config_path = config_path
    self.robot_config_file = robot_config_file
    self.world = world
    self.use_image_features = use_image_features
    self.groundtruth_bagfile = groundtruth_bagfile


def load_params(param_file):
  graph_bag_params_list = []
  with open(param_file) as param_csvfile:
    reader = csv.reader(param_csvfile, delimiter=' ')
    for row in reader:
      graph_bag_params_list.append(GraphBagParams(row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]))

  return graph_bag_params_list


def combine_results_in_csv_file(graph_bag_params, output_dir):
  # Don't save this as *stats.csv otherwise it will be including when averaging bag results in average_results.py
  combined_results_csv_file = os.path.join(output_dir, 'bag_sweep_stats_combined.csv')
  output_csv_files = []
  bag_names = []
  for params in graph_bag_params:
    bag_name = os.path.splitext(os.path.basename(params.bagfile))[0]
    bag_names.append(bag_name)
    output_csv_files.append(os.path.join(output_dir, bag_name + '_stats.csv'))
  combined_dataframe = average_results.combined_results(output_csv_files)
  combined_dataframe.insert(0, 'Bag', bag_names)
  combined_dataframe.to_csv(combined_results_csv_file, index=False)
  return combined_results_csv_file


def check_params(graph_bag_params_list):
  for params in graph_bag_params_list:
    if not os.path.isfile(params.bagfile):
      print('Bagfile ' + params.bagfile + ' does not exist.')
      sys.exit()
    if not os.path.isfile(params.map_file):
      print('Map file ' + params.map_file + ' does not exist.')
      sys.exit()
    if not os.path.isfile(params.groundtruth_bagfile):
      print('Bagfile ' + params.groundtruth_bagfile + ' does not exist.')
      sys.exit()


# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@multiprocessing_helpers.full_traceback
def run_graph_bag(params, output_dir):
  bag_name = os.path.splitext(os.path.basename(params.bagfile))[0]
  output_bag_path = os.path.join(output_dir, bag_name + '_results.bag')
  output_csv_file = os.path.join(output_dir, bag_name + '_stats.csv')
  run_command = 'rosrun graph_bag run_graph_bag ' + params.bagfile + ' ' + params.map_file + ' ' + params.config_path + ' -i ' + params.image_topic + ' -o ' + output_bag_path + ' -r ' + params.robot_config_file + ' -w ' + params.world + ' -s ' + output_csv_file + ' -f ' + params.use_image_features
  os.system(run_command)
  output_pdf_file = os.path.join(output_dir, bag_name + '_output.pdf')
  plot_command = 'rosrun graph_bag plot_results_main.py ' + output_bag_path + ' --output-file ' + output_pdf_file + ' --output-csv-file ' + output_csv_file + ' -g ' + params.groundtruth_bagfile
  os.system(plot_command)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def run_graph_bag_helper(zipped_vals):
  return run_graph_bag(*zipped_vals)


def bag_sweep(config_file, output_dir):
  graph_bag_params_list = load_params(config_file)
  check_params(graph_bag_params_list)
  num_processes = 6
  pool = multiprocessing.Pool(num_processes)
  # izip arguments so we can pass as one argument to pool worker
  pool.map(run_graph_bag_helper, itertools.izip(graph_bag_params_list, itertools.repeat(output_dir)))
  combined_results_csv_file = combine_results_in_csv_file(graph_bag_params_list, output_dir)
  output_file = os.path.join(output_dir, 'bag_sweep_results.pdf')
  plot_bag_sweep_results.create_plot(output_file, combined_results_csv_file)
