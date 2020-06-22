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

import os

import plot_creator
import utilities

def get_subdirectory_files(file_string):
  subdirectory_csv_files = []
  _, subdirectories, _ = os.walk(os.getcwd()).next()
  for subdirectory in subdirectories:
    subdirectory_path = os.path.join(os.getcwd(), subdirectory)
    for subdirectory_csv_file in utilities.get_files(subdirectory_path, file_string):
      subdirectory_csv_files.append(subdirectory_csv_file)
  return subdirectory_csv_files

def remove_repeat_job_id_rows(dataframe):
  return dataframe.drop_duplicates('job_id') 

def average_same_jobs(dataframe):
  return dataframe.groupby(['job_id']).mean() 

# Averages results from all *results.csv files in subdirectories in a directory.  Only looks for files at a depth of one (one subdirectory, doesn't search recursively)
if __name__ == '__main__':
  dataframes = []

  results_csv_files = get_subdirectory_files('*results.csv')
  results_dataframe = plot_creator.load_dataframe(results_csv_files)
  averaged_results_dataframe = average_same_jobs(results_dataframe)
  averaged_results_dataframe.reset_index(inplace=True)
  dataframes.append(averaged_results_dataframe)

  values_csv_files = get_subdirectory_files('*values.csv')
  values_dataframe = plot_creator.load_dataframe(values_csv_files)
  values_dataframe.columns.name = 'values'
  values_dataframe = remove_repeat_job_id_rows(values_dataframe)
  dataframes.append(values_dataframe)
  a = values_dataframe['job_id']

  pdf_filename = 'averaged_result_plots.pdf'
  plot_creator.create_pdf(dataframes, pdf_filename, True, os.path.basename(os.getcwd()))
