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

import csv
import itertools
import os

import average_results


def concat_results(job_ids, directory, stats_filename):
    results_csv_files = []
    for job_id in job_ids:
        results_csv_files.append(os.path.join(directory, str(job_id), stats_filename))
    # Results are written in job id order
    combined_results = average_results.combined_results(results_csv_files)
    combined_results_file = os.path.join(directory, "param_sweep_combined_results.csv")
    combined_results.to_csv(combined_results_file, index=False)


def make_all_value_combinations(value_ranges):
    return list(itertools.product(*value_ranges))


def save_values(value_names, values, filename, output_dir):
    with open(os.path.join(output_dir, filename), "w") as values_file:
        writer = csv.writer(values_file)
        writer.writerow(value_names)
        writer.writerows(values)
