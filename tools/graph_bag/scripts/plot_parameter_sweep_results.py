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
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd

import os
import sys


def create_plot(output_file, csv_file, value_combos_file):
  dataframe = pd.read_csv(csv_file)
  rmses = dataframe['rmse']
  x_axis_vals = []
  x_axis_label = ''
  if value_combos_file:
    value_combos_dataframe = pd.read_csv(value_combos_file)
    if (len(value_combos_dataframe.columns) > 1):
      print('Value combos include more than one parameter, cannot use for x axis of plot')
      exit()
    x_axis_label = value_combos_dataframe.columns[0]
    x_axis_vals = value_combos_dataframe[x_axis_label]
  else:
    job_count = dataframe.shape[0]
    x_axis_vals = range(job_count)
    x_axis_label = 'Job Id'
  with PdfPages(output_file) as pdf:
    plt.figure()
    plt.plot(x_axis_vals, rmses, linestyle='None', marker='o', markeredgewidth=0.1, markersize=10.5)
    plt.xlabel(x_axis_label)
    plt.ylabel('RMSE')
    plt.title('RMSE vs. ' + x_axis_label)
    x_range = x_axis_vals[len(x_axis_vals) - 1] - x_axis_vals[0]
    x_buffer = x_range * 0.1
    # Extend x axis on either side to make data more visible
    plt.xlim([x_axis_vals[0] - x_buffer, x_axis_vals[len(x_axis_vals) - 1] + x_buffer])
    plt.ticklabel_format(useOffset=False)
    plt.tight_layout()
    pdf.savefig()
    plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('csv_file')
  parser.add_argument('--output-file', default='param_sweep_results.pdf')
  parser.add_argument('--value-combos-file')
  args = parser.parse_args()
  create_plot(args.output_file, args.csv_file, args.value_combos_file)
