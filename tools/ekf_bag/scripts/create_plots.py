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
import os

import plot_creator
import utilities

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--directory", default=None, help="Directory where results csv files are stored"
    )
    parser.add_argument(
        "--write_values",
        "-v",
        action="store_true",
        help="Write parameter sweep values to a plot in pdf",
    )
    parser.add_argument(
        "--write_values_table",
        "-t",
        action="store_true",
        help="Write parameter sweep values to a table in pdf",
    )

    args = parser.parse_args()
    if args.write_values_table and not args.write_values:
        print("If write_values_table enabled, write_values must be as well.")
        exit()

    directory = args.directory
    if directory == None:
        directory = os.getcwd()

    dataframes = []

    results_filestring = "*results.csv"
    results_files = utilities.get_files(directory, results_filestring)
    if len(results_files) == 0:
        print(("No results csv files found in directory " + directory))
        exit()

    dataframes.append(plot_creator.load_dataframe(results_files))

    if args.write_values:
        values_filestring = "*values.csv"
        values_files = utilities.get_files(directory, values_filestring)
        values_dataframe = plot_creator.load_dataframe(values_files)
        values_dataframe.columns.name = "values"
        dataframes.append(values_dataframe)

    pdf_filename = "result_plots.pdf"
    plot_creator.create_pdf(
        dataframes, pdf_filename, args.write_values_table, os.path.basename(directory)
    )
