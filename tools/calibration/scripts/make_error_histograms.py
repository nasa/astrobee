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

import argparse
import csv
import os
import sys

import matplotlib.pyplot as plt


def make_histograms(errors_file):
  x_errors = []
  y_errors = []
  error_norms = []
  with open(errors_file) as errors_csvfile:
    reader = csv.reader(errors_csvfile, delimiter=" ")
    for row in reader:
      x_error = float(row[0])
      y_error = float(row[1])
      error_norm = x_error * x_error + y_error * y_error
      x_errors.append(x_error)
      y_errors.append(y_error)
      error_norms.append(error_norm)

  plt.hist(x_errors, bins=100)
  plt.ylabel('Count')
  plt.xlabel('X Errors')
  plt.savefig('x_errors_histogram.png')
  plt.clf()

  plt.hist(y_errors, bins=100)
  plt.ylabel('Count')
  plt.xlabel('Y Errors')
  plt.savefig('y_errors_histogram.png')
  plt.clf()

  plt.hist(error_norms, bins=100)
  plt.ylabel('Count')
  plt.xlabel('Error Norms')
  plt.savefig('error_norms_histogram.png')
  plt.clf()


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("errors_file")
  args = parser.parse_args()
  if not os.path.isfile(args.errors_file):
    print('Errors file not found.')
    sys.exit()
  make_histograms(args.errors_file)
