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
"""
Creates a new bagfile with trimmed start and end times from a provided bagfile.
"""

import argparse
import os
import sys

import rosbag
import utilities.utilities


def trim_bag(bag_name, start_time_to_trim, end_time_to_trim):
    with rosbag.Bag(bag_name, "r") as bag:
        start_time = bag.get_start_time()
        new_start_time = start_time + start_time_to_trim
        end_time = bag.get_end_time()
        new_end_time = end_time - end_time_to_trim
        output_bag_name = os.path.splitext(bag_name)[0] + "_trimmed.bag"
        run_command = (
            "rosbag filter "
            + bag_name
            + " "
            + output_bag_name
            + ' "t.secs >= '
            + str(new_start_time)
            + " and t.secs <= "
            + str(new_end_time)
            + '"'
        )
        os.system(run_command)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument(
        "-s",
        "--start-time-to-trim",
        type=float,
        default=0,
        help="Duration of time to trim from the beginning of the bagfile in seconds.",
    )
    parser.add_argument(
        "-e",
        "--end-time-to-trim",
        type=float,
        default=0,
        help="Duration of time to trim from the end of the bagfile in seconds.",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    trim_bag(args.bagfile, args.start_time_to_trim, args.end_time_to_trim)
