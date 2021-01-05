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
import pandas as pd

import os
import sys


def add_job_id_mean_row(job_id, dataframes, mean_dataframe, names):
  combined_dataframe = pd.DataFrame(None, None, names)
  for dataframe in dataframes:
    trimmed_dataframe = pd.DataFrame(dataframe.values[job_id: job_id + 1], columns=names)
    combined_dataframe = combined_dataframe.append(trimmed_dataframe, ignore_index=True)
  job_id_mean_dataframe = pd.DataFrame(None, None, names)
  for name in names:
    job_id_mean_dataframe[name] = [combined_dataframe[name].mean()]
  return mean_dataframe.append(job_id_mean_dataframe, ignore_index=True)

def average_parameter_sweep_results(combined_results_csv_files, directory):
  dataframes = [pd.read_csv(file) for file in combined_results_csv_files]
  if not dataframes:
    print('Failed to create dataframes')
    exit()
  names = dataframes[0].columns
  mean_dataframe = pd.DataFrame(None, None, names)
  jobs = dataframes[0].shape[0]
  # Save job id mean in order, so nth row coressponds to nth job id in final mean dataframe
  for job_id in range(jobs):
    mean_dataframe = add_job_id_mean_row(job_id, dataframes, mean_dataframe, names)
  mean_dataframe_file = os.path.join(directory, 'bag_and_param_sweep_stats.csv')
  mean_dataframe.to_csv(mean_dataframe_file, index=False)


def bag_and_parameter_sweep(graph_bag_params_list, output_dir):
  combined_results_csv_files = []
  for graph_bag_params in graph_bag_params_list:
    # Save parameter sweep output in different directory for each bagfile, name directory using bagfile
    bag_name_prefix = os.path.splitext(os.path.basename(graph_bag_params.bagfile))[0]
    bag_output_dir = os.path.join(output_dir, bag_name_prefix)
    parameter_sweep.make_values_and_parameter_sweep(bag_output_dir, graph_bag_params.bagfile, graph_bag_params.map_file, graph_bag_params.image_topic, graph_bag_params.config_path, graph_bag_params.robot_config_file, graph_bag_params.world, graph_bag_params.use_image_features)
    combined_results_csv_files.append(os.path.join(bag_output_dir, 'param_sweep_combined_results.csv'))
  average_parameter_sweep_results(combined_results_csv_files, output_dir)

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

