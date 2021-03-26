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
import csv
import itertools
import math
import multiprocessing
import numpy as np
import os

import average_results
import config_creator
import plot_parameter_sweep_results
import utilities


# Run graph localizer with values.
# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@utilities.full_traceback
def test_values(values, job_id, value_names, output_dir, bag_file, map_file, image_topic, config_path, robot_config,
                world, use_image_features, groundtruth_bagfile, rmse_rel_start_time, rmse_rel_end_time):
  new_output_dir = os.path.join(output_dir, str(job_id))
  os.mkdir(new_output_dir)
  graph_config_filepath = os.path.join(config_path, "config", "graph_localizer.config")
  new_graph_config_filepath = os.path.join(new_output_dir, "graph_localizer.config")
  config_creator.make_config(values, value_names, graph_config_filepath, new_graph_config_filepath)
  output_bag = os.path.join(new_output_dir, "results.bag")
  output_stats_file = os.path.join(new_output_dir, "graph_stats.csv")
  graph_config_prefix = new_output_dir + '/'
  run_command = 'rosrun graph_bag run_graph_bag ' + bag_file + ' ' + map_file + ' ' + config_path + ' -o ' + output_bag + ' -s ' + output_stats_file + ' -r ' + robot_config + ' -w ' + world + ' -g ' + graph_config_prefix + ' -f ' + str(
    use_image_features)
  if image_topic is not None:
    run_command += ' -i ' + image_topic
  os.system(run_command)
  output_pdf_file = os.path.join(new_output_dir, str(job_id) + '_output.pdf')
  output_csv_file = os.path.join(new_output_dir, 'graph_stats.csv')
  plot_command = 'rosrun graph_bag plot_results_main.py ' + output_bag + ' --output-file ' + output_pdf_file + ' --output-csv-file ' + output_csv_file + ' -g ' + groundtruth_bagfile + ' --rmse-rel-start-time ' + str(
    rmse_rel_start_time) + ' --rmse-rel-end-time ' + str(rmse_rel_end_time)
  os.system(plot_command)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def test_values_helper(zipped_vals):
  return test_values(*zipped_vals)


def concat_results(job_ids, directory):
  results_csv_files = []
  for job_id in job_ids:
    results_csv_files.append(os.path.join(directory, str(job_id), 'graph_stats.csv'))
  # Results are written in job id order
  combined_results = average_results.combined_results(results_csv_files)
  combined_results_file = os.path.join(directory, 'param_sweep_combined_results.csv')
  combined_results.to_csv(combined_results_file, index=False)


def parameter_sweep(all_value_combos,
                    value_names,
                    output_dir,
                    bag_file,
                    map_file,
                    image_topic,
                    config_path,
                    robot_config,
                    world,
                    use_image_features,
                    groundtruth_bagfile,
                    rmse_rel_start_time=0,
                    rmse_rel_end_time=-1):
  job_ids = list(range(len(all_value_combos)))
  num_processes = 6
  pool = multiprocessing.Pool(num_processes)
  # izip arguments so we can pass as one argument to pool worker
  pool.map(
    test_values_helper,
    itertools.izip(all_value_combos, job_ids, itertools.repeat(value_names), itertools.repeat(output_dir),
                   itertools.repeat(bag_file), itertools.repeat(map_file), itertools.repeat(image_topic),
                   itertools.repeat(config_path), itertools.repeat(robot_config), itertools.repeat(world),
                   itertools.repeat(use_image_features), itertools.repeat(groundtruth_bagfile),
                   itertools.repeat(rmse_rel_start_time), itertools.repeat(rmse_rel_end_time)))
  concat_results(job_ids, output_dir)


def make_all_value_combinations(value_ranges):
  return list(itertools.product(*value_ranges))


def make_value_ranges():
  value_ranges = []
  value_names = []
  steps = 10 

  # tune num smart factors
  #value_ranges.append(np.logspace(-1, -6, steps, endpoint=True))
  #value_names.append('accel_bias_sigma')
  value_ranges.append(np.linspace(0, 500, steps, endpoint=True))
  value_names.append('smart_projection_adder_feature_track_min_separation')

  #q_gyro
  # .001 -> 2 deg
  #q_gyro_degrees_range = np.logspace(-3, .3, steps, endpoint=True)
  #q_gyro_squared_rads_range = [math.radians(deg)**2 for deg in q_gyro_degrees_range]
  #value_ranges.append(q_gyro_squared_rads_range)
  #value_names.append('q_gyro')

  return value_ranges, value_names


def save_values(value_names, values, filename, output_dir):
  with open(os.path.join(output_dir, filename), 'w') as values_file:
    writer = csv.writer(values_file)
    writer.writerow(value_names)
    writer.writerows(values)


def make_values_and_parameter_sweep(output_dir,
                                    bag_file,
                                    map_file,
                                    image_topic,
                                    config_path,
                                    robot_config,
                                    world,
                                    use_image_features,
                                    groundtruth_bagfile,
                                    rmse_rel_start_time=0,
                                    rmse_rel_end_time=-1):
  output_dir = utilities.create_directory(output_dir)
  print('Output directory for results is {}'.format(output_dir))

  value_ranges, value_names = make_value_ranges()
  save_values(value_names, value_ranges, 'individual_value_ranges.csv', output_dir)

  all_value_combos = make_all_value_combinations(value_ranges)
  save_values(value_names, all_value_combos, 'all_value_combos.csv', output_dir)

  parameter_sweep(all_value_combos, value_names, output_dir, bag_file, map_file, image_topic, config_path, robot_config,
                  world, use_image_features, groundtruth_bagfile, rmse_rel_start_time, rmse_rel_end_time)
  combined_results_file = os.path.join(output_dir, 'param_sweep_combined_results.csv')
  value_combos_file = os.path.join(output_dir, 'all_value_combos.csv')
  results_pdf_file = os.path.join(output_dir, 'param_sweep_results.pdf')
  plot_parameter_sweep_results.create_plots(results_pdf_file, combined_results_file, value_combos_file)
  return output_dir


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bag_file', help='Full path to bagfile.')
  parser.add_argument('map_file', help='Full path to map file.')
  parser.add_argument('image_topic', help='Image topic.')
  parser.add_argument('config_path', help='Full path to config path.')
  parser.add_argument('robot_config', help='Relative path to robot config.')
  parser.add_argument('world', help='World being used.')
  parser.add_argument('--generate-image-features',
                      dest='use_image_features',
                      action='store_false',
                      help='Use image features msgs from bagfile or generate features from images.')

  parser.add_argument('-g', '--groundtruth-bagfile', default=None)
  parser.add_argument(
    '--directory',
    default=None,
    help=
    'Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.'
  )
  args = parser.parse_args()
  # Default set groundtruth bagfile to normal bagfile
  if not args.groundtruth_bagfile:
    args.groundtruth_bagfile = args.bag_file

  make_values_and_parameter_sweep(args.directory, args.bag_file, args.map_file, args.image_topic, args.config_path,
                                  args.robot_config, args.world, args.use_image_features, args.groundtruth_bagfile)
