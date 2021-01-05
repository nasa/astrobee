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

def create_plot(output_file, csv_file):
  dataframe = pd.read_csv(csv_file)
  rmses = dataframe['rmse']
  job_count = dataframe.shape[0]
  job_ids = range(job_count) 
  with PdfPages(output_file) as pdf:
    plt.figure()
    plt.plot(job_ids, rmses, linestyle='None',
                              marker='o',
                              markeredgewidth=0.1,
                              markersize=10.5) 
    plt.xlabel('Job Id')
    plt.ylabel('RMSE')
    plt.title('RMSE for Job Ids')
    # Extend x axis on either side to make data more visible
    plt.xlim([-1, job_count + 1])
    plt.ticklabel_format(useOffset=False)
    pdf.savefig()
    plt.close()
 

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('csv_file')
  parser.add_argument('--output-file', default='param_sweep_results.pdf')
  args = parser.parse_args()
  create_plot(args.output_file, args.csv_file)
