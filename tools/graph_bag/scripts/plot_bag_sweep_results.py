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


def create_plot(output_file, csv_file, label_1='', csv_file_2=None, label_2=''):
  dataframe = pd.read_csv(csv_file)
  dataframe.sort_values(by=['Bag'], inplace=True)
  rmses = dataframe['rmse']
  bag_names = dataframe['Bag'].tolist()
  max_name_length = 45
  shortened_bag_names = [
    bag_name[-1 * max_name_length:] if len(bag_name) > max_name_length else bag_name for bag_name in bag_names
  ]
  x_axis_vals = range(len(shortened_bag_names))
  rmses_2 = None
  if (csv_file_2):
    dataframe_2 = pd.read_csv(csv_file_2)
    dataframe_2.sort_values(by=['Bag'], inplace=True)
    rmses_2 = dataframe_2['rmse']
    bag_names_2 = dataframe_2['Bag'].tolist()
    if bag_names != bag_names_2:
      print('Bag names for first and second csv file are not the same')
      exit()
  with PdfPages(output_file) as pdf:
    plt.figure()
    plt.plot(x_axis_vals, rmses, 'b', label=label_1, linestyle='None', marker='o', markeredgewidth=0.1, markersize=10.5)
    if (csv_file_2):
      plt.plot(x_axis_vals,
               rmses_2,
               'r',
               label=label_2,
               linestyle='None',
               marker='o',
               markeredgewidth=0.1,
               markersize=10.5)
      plt.legend(prop={'size': 8}, bbox_to_anchor=(1.05, 1))
    plt.xticks(x_axis_vals, shortened_bag_names, fontsize=7, rotation=20)
    plt.ylabel('RMSE')
    plt.title('RMSE vs. Bag')
    x_range = x_axis_vals[len(x_axis_vals) - 1] - x_axis_vals[0]
    x_buffer = x_range * 0.1
    # Extend x axis on either side to make data more visible
    plt.xlim([x_axis_vals[0] - x_buffer, x_axis_vals[len(x_axis_vals) - 1] + x_buffer])
    plt.tight_layout()
    pdf.savefig()
    plt.close()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  # Combined csv results, where each row is the result from a bag file
  parser.add_argument('csv_file')
  parser.add_argument('--output-file', default='bag_sweep_results.pdf')
  parser.add_argument('--csv-file2', help='Optional second csv file to plot')
  parser.add_argument('--label1', default='', help='Optional label for first csv file')
  parser.add_argument('--label2', default='', help='Optional label for second csv file')
  args = parser.parse_args()
  create_plot(args.output_file, args.csv_file, args.label1, args.csv_file2, args.label2)
