#!/usr/bin/python3
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

import matplotlib.pyplot as plt

import rmse_calculators


# Computes and saves rmse for poses compared to groundtruth poses.
# Optionally pass the max allowed time difference between poses defined to be
# at the same timestamp, along with the start and end time bounds for computing
# same timestamp poses (passing an end time of -1 is equivalent to having no upper bound time).
def save_rmse(
    poses,
    groundtruth_poses,
    csv_file,
    pdf,
    prefix="",
    max_allowed_time_diff=0.01,
    start_time=0,
    end_time=-1,
):
    rmse = rmse_calculators.pose_rmse(
        poses, groundtruth_poses, max_allowed_time_diff, start_time, end_time
    )

    stats = (
        prefix
        + " pos rmse: "
        + str(rmse[0])
        + "\n"
        + "orientation rmse: "
        + str(rmse[1])
    )
    with open(csv_file, "a") as output_csv:
        csv_writer = csv.writer(output_csv, lineterminator="\n")
        csv_writer.writerow([prefix + " rmse", " " + str(rmse[0])])
        csv_writer.writerow([prefix + " Orientation rmse", " " + str(rmse[1])])
    plt.figure()
    plt.axis("off")
    plt.text(0.0, 0.5, stats)
    pdf.savefig()
