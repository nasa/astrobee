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

matplotlib.use("pdf")
import math
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages


def create_plot(pdf, csv_file, value_combos_file, prefix=""):
    dataframe = pd.read_csv(csv_file)
    try:
        rmses = dataframe[prefix + "rmse"]
    except:
        return
    x_axis_vals = []
    x_axis_label = ""
    if value_combos_file:
        value_combos_dataframe = pd.read_csv(value_combos_file)
        if len(value_combos_dataframe.columns) > 1:
            print(
                "Value combos include more than one parameter, cannot use for x axis of plot"
            )
            job_count = dataframe.shape[0]
            x_axis_vals = list(range(job_count))
            x_axis_label = "Job Id"
        else:
            x_axis_label = value_combos_dataframe.columns[0]
            x_axis_vals = value_combos_dataframe[x_axis_label]
            if isinstance(x_axis_vals[0], str):
                job_count = dataframe.shape[0]
                x_axis_vals = list(range(job_count))
                x_axis_label = "Job Id"

    else:
        job_count = dataframe.shape[0]
        x_axis_vals = list(range(job_count))
        x_axis_label = "Job Id"

    plt.figure()
    plt.plot(
        x_axis_vals,
        rmses,
        linestyle="None",
        marker="o",
        markeredgewidth=0.1,
        markersize=10.5,
    )
    plt.ylabel(prefix + " RMSE")
    plt.title(prefix + " RMSE vs. " + x_axis_label)
    x_range = x_axis_vals[len(x_axis_vals) - 1] - x_axis_vals[0]
    first_x_val = x_axis_vals[0]
    last_x_val = x_axis_vals[len(x_axis_vals) - 1]
    # Use log scale if min and max x vals are more than 3 orders of magnitude apart
    if (
        first_x_val != 0
        and last_x_val != 0
        and abs(math.log10(last_x_val) - math.log10(first_x_val)) > 3
    ):
        plt.xscale("log", basex=10)
        # Extend x axis on either side using a log scale to make data more visible
        if first_x_val < last_x_val:
            plt.xlim([first_x_val * 0.1, last_x_val * 10.0])
        else:
            plt.xlim([last_x_val * 0.1, first_x_val * 10.0])
    else:
        # Extend x axis on either side to make data more visible
        x_buffer = x_range * 0.1
        plt.xlim(
            [x_axis_vals[0] - x_buffer, x_axis_vals[len(x_axis_vals) - 1] + x_buffer]
        )
        plt.ticklabel_format(useOffset=False)
    plt.tight_layout()
    plt.xlabel(x_axis_label)
    pdf.savefig()
    plt.close()


def create_plots(output_file, csv_file, value_combos_file):
    with PdfPages(output_file) as pdf:
        # Graph RMSEs
        create_plot(pdf, csv_file, value_combos_file)
        create_plot(pdf, csv_file, value_combos_file, "orientation_")
        create_plot(pdf, csv_file, value_combos_file, "integrated_")
        create_plot(pdf, csv_file, value_combos_file, "rel_")
        create_plot(pdf, csv_file, value_combos_file, "rel_orientation_")
        create_plot(pdf, csv_file, value_combos_file, "rel_integrated_")
        # IMU Augmented RMSEs
        create_plot(pdf, csv_file, value_combos_file, "imu_augmented_")
        create_plot(pdf, csv_file, value_combos_file, "imu_augmented_orientation_")
        create_plot(pdf, csv_file, value_combos_file, "imu_augmented_integrated_")
        create_plot(pdf, csv_file, value_combos_file, "rel_imu_augmented_")
        create_plot(pdf, csv_file, value_combos_file, "rel_imu_augmented_orientation_")
        create_plot(pdf, csv_file, value_combos_file, "rel_imu_augmented_integrated_")

        # IMU Bias Tester RMSEs
        create_plot(pdf, csv_file, value_combos_file, "imu_bias_tester_")
        create_plot(pdf, csv_file, value_combos_file, "imu_bias_tester_orientation_")
        create_plot(pdf, csv_file, value_combos_file, "rel_imu_bias_tester_")
        create_plot(
            pdf, csv_file, value_combos_file, "rel_imu_bias_tester_orientation_"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # Csv file with combined results (or mean results if using bag_and_param_sweep) for each job
    parser.add_argument("csv_file")
    parser.add_argument("--output-file", default="param_sweep_results.pdf")
    parser.add_argument("--value-combos-file")
    args = parser.parse_args()
    create_plots(args.output_file, args.csv_file, args.value_combos_file)
