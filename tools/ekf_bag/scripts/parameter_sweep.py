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

import config_creator
import ekf_graph
import utilities


# Run ekf with values.
# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@utilities.full_traceback
def test_values(values, job_id, value_names, output_dir, bag_file, map_file, image_topic, gnc_config):
  new_gnc_config_filepath = os.path.join(output_dir, str(job_id) + "_gnc.config")
  config_creator.make_config(values, value_names, gnc_config, new_gnc_config_filepath)

  ekf_output_file = os.path.join(output_dir, str(job_id) + "_ekf_results.txt")
  pdf_output_file = os.path.join(output_dir, str(job_id) + "_plots.pdf")
  results_csv_output_file = os.path.join(output_dir, str(job_id) + "_results.csv")

  values_file_name = str(job_id) + '_values.csv'
  with open(os.path.join(output_dir, values_file_name), 'w') as values_file:
    id_and_values = (job_id,) + values
    id_name_and_value_names = ['job_id'] + value_names
    writer = csv.writer(values_file)
    writer.writerow(id_name_and_value_names)
    writer.writerow(id_and_values)

  options = ekf_graph.RunEKFOptions(bag_file, map_file, ekf_output_file, pdf_output_file, results_csv_output_file)
  options.set_param_sweep_params(image_topic, job_id, new_gnc_config_filepath)
  ekf_graph.run_ekf_and_save_stats(options)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def test_values_helper(zipped_vals):
  return test_values(*zipped_vals)


def parameter_sweep(all_value_combos, value_names, output_dir, bag_file, map_file, image_topic, gnc_config):
  job_ids = list(range(len(all_value_combos)))
  num_processes = 6
  pool = multiprocessing.Pool(num_processes)
  # izip arguments so we can pass as one argument to pool worker
  pool.map(
    test_values_helper,
    itertools.izip(all_value_combos, job_ids, itertools.repeat(value_names), itertools.repeat(output_dir),
                   itertools.repeat(bag_file), itertools.repeat(map_file), itertools.repeat(image_topic),
                   itertools.repeat(gnc_config)))


def make_all_value_combinations(value_ranges):
  return list(itertools.product(*value_ranges))


def make_value_ranges():
  value_ranges = []
  value_names = []
  steps = 50

  #tune_ase_Q_imu

  #q_gyro
  # .001 -> 2 deg
  #q_gyro_degrees_range = np.logspace(-3, .3, steps, endpoint=True)
  #q_gyro_squared_rads_range = [math.radians(deg)**2 for deg in q_gyro_degrees_range]
  #value_ranges.append(q_gyro_squared_rads_range)
  #value_names.append('q_gyro')

  #q_gyro_bias
  #q_gyro_bias_degrees_range = np.logspace(-8, 0, steps, endpoint=True)
  #q_gyro_bias_squared_rads_range = [math.radians(bias)**2 for bias in q_gyro_bias_degrees_range]
  #value_ranges.append(q_gyro_bias_squared_rads_range)
  #value_names.append('q_gyro_bias')

  #q_accel (logspace)
  #value_ranges.append(list(np.logspace(-9, -5, steps, endpoint=True)))
  #value_names.append('q_accel')

  #q_accel_bias
  #value_ranges.append(list(np.logspace(-11, -5, steps, endpoint=True)))
  #value_names.append('q_accel_bias')

  #Tune Image Params

  #tun_ase_map_error
  #value_ranges.append(list(np.linspace(0, 0.1, steps, endpoint=True)))
  #value_names.append('tun_ase_map_error')

  #tun_ase_vis_r_mag
  #value_ranges.append(list(np.linspace(2, 6, steps, endpoint=True)))
  #value_names.append('tun_ase_vis_r_mag')
  #tun_ase_of_r_mag
  #value_ranges.append(list(np.linspace(0, 3, steps, endpoint=True)))
  #value_names.append('tun_ase_of_r_mag')

  #tun_ase_dock_r_mag
  #value_ranges.append(list(np.linspace(1, 3, steps, endpoint=True)))
  #value_names.append('tun_ase_dock_r_mag')

  # Other Options

  # tun_ase_mahal_distance_max
  value_ranges.append(list(np.linspace(0, 40, steps, endpoint=True)))
  value_names.append('tun_ase_mahal_distance_max')

  return value_ranges, value_names


def save_values(value_names, values, filename, output_dir):
  with open(os.path.join(output_dir, filename), 'w') as values_file:
    writer = csv.writer(values_file)
    writer.writerow(value_names)
    writer.writerows(values)

def make_values_and_parameter_sweep(output_dir, bag_file, map_file, image_topic, gnc_config):
  output_dir = utilities.create_directory(output_dir)
  print('Output directory for results is {}'.format(output_dir))

  value_ranges, value_names = make_value_ranges()
  save_values(value_names, value_ranges, 'individual_value_ranges.csv', output_dir)

  all_value_combos = make_all_value_combinations(value_ranges)
  save_values(value_names, all_value_combos, 'all_value_combos.csv', output_dir)

  parameter_sweep(all_value_combos, value_names, output_dir, bag_file, map_file, image_topic, gnc_config)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bag_file', help='Full path to bagfile.')
  parser.add_argument('map_file', help='Full path to map file.')

  parser.add_argument(
    '--directory',
    default=None,
    help=
    'Full path to output directory where files will be saved. If not specified, timestamped directory will be created in current path.'
  )
  parser.add_argument('--image_topic',
                      '-i',
                      default=None,
                      help='Image topic for images in bagfile. Default topic will be used if not specified.')

  parser.add_argument(
    '--gnc_config',
    '-g',
    default=None,
    help=
    'GNC config to use as a base for the parameter sweep.  Parameters not swept over will be taken from this file. Default will be gnc.config in astrobee/config'
  )
  #TODO(rsoussan): pass number of processes to run and parallel and parameter ranges as arguments

  args = parser.parse_args()

  gnc_config = utilities.get_gnc_config(args.gnc_config)

  make_values_and_parameter_sweep(args.directory, args.bag_file, args.map_file, args.image_topic, gnc_config)
