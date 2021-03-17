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

import argparse
import itertools
import multiprocessing
import os
import pandas as pd

import ekf_graph
import utilities


# Run ekf on bag file.
# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@utilities.full_traceback
def test_on_bag(bag_file, output_dir, args):
  print("Running EKF test on {}".format(bag_file))
  name_prefix = os.path.splitext(os.path.basename(bag_file))[0]
  ekf_output_file = os.path.join(output_dir, name_prefix + '_ekf.txt')
  pdf_output_file = os.path.join(output_dir, name_prefix + '_plots.pdf')
  results_csv_output_file = os.path.join(output_dir, name_prefix + '_results.csv')
  options = ekf_graph.RunEKFOptions(bag_file, args.map_file, ekf_output_file, pdf_output_file, results_csv_output_file)
  options.set_bag_sweep_params(args)
  options.features_in_bag = True
  ekf_graph.run_ekf_and_save_stats(options)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def test_on_bag_helper(zipped_vals):
  return test_on_bag(*zipped_vals)


#TODO: replace args with individual params
def bag_sweep(bag_files, output_dir, args):
  num_processes = 6
  pool = multiprocessing.Pool(num_processes)
  # izip arguments so we can pass as one argument to pool worker
  pool.map(test_on_bag_helper, itertools.izip(bag_files, itertools.repeat(output_dir), itertools.repeat(args)))


def combined_results(csv_files):
  dataframes = [pd.read_csv(file) for file in csv_files]
  if not dataframes:
    print('Failed to create dataframes')
    exit()
  names = dataframes[0].columns
  combined_dataframes = pd.DataFrame(None, None, names)
  for dataframe in dataframes:
    trimmed_dataframe = pd.DataFrame(dataframe.values[0:1], columns=names)
    combined_dataframes = combined_dataframes.append(trimmed_dataframe, ignore_index=True)
  return combined_dataframes


def combine_results_in_csv_file(bag_files, output_dir):
  # Don't save this as *stats.csv otherwise it will be including when averaging bag results in average_results.py
  combined_results_csv_file = os.path.join(output_dir, 'bag_sweep_stats_combined.csv')
  output_csv_files = []
  for bag_file in bag_files:
    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    output_csv_files.append(os.path.join(output_dir, bag_name + '_results.csv'))
  combined_dataframe = combined_results(output_csv_files)
  combined_dataframe.to_csv(combined_results_csv_file, index=False)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('map_file', help='Full path to map file.')
  parser.add_argument('gnc_config', help='Full path to gnc config file.')
  parser.add_argument('-d', '--bag_directory', default=None, help='Full path to directory containing bag files.')
  parser.add_argument(
    '-o',
    '--output_directory',
    default=None,
    help=
    'Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.'
  )

  parser.add_argument('-r', '--robot_name', metavar='ROBOT', help='Specify the robot to use (just name, not path).')
  parser.add_argument('-i', '--image_topic', dest='image_topic', default=None, help='Use specified image topic.')
  parser.add_argument('--save_stats', action='store_true', help='Save stats to csv file.')
  parser.add_argument('--make_plots', type=bool, default=True, help='Make pdf of plots of results.')

  args, args_unkonown = parser.parse_known_args()

  bag_directory = args.bag_directory
  if bag_directory == None:
    bag_directory = os.getcwd()

  if not os.path.exists(bag_directory):
    print("Bag directory {} does not exist.".format(bag_directory))
    exit()
  bag_files = utilities.get_files(bag_directory, '*.bag')
  print("Found {} bag files in {}.".format(len(bag_files), bag_directory))

  output_dir = utilities.create_directory(args.output_directory)
  print('Output directory for results is {}'.format(output_dir))

  bag_sweep(bag_files, output_dir, args)
  combine_results_in_csv_file(bag_files, output_dir)
