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
import sys

import imu_analyzer

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile")
    parser.add_argument("--output-file", default="imu_analyzer_output.pdf")
    parser.add_argument("-f", "--filtered-bagfile", default="")
    parser.add_argument("-s", "--sample-rate", type=float, default=62.5)
    # Only applicable if filtered_bagfile not provided, uses python filters
    parser.add_argument("-c", "--cutoff-frequency", type=float, default=5.0)
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()
    if args.filtered_bagfile and not os.path.isfile(args.filtered_bagfile):
        print(("Bag file " + args.filtered_bagfile + " does not exist."))
        sys.exit()
    imu_analyzer.create_plots(
        args.bagfile,
        args.filtered_bagfile,
        args.output_file,
        args.cutoff_frequency,
        args.sample_rate,
    )
