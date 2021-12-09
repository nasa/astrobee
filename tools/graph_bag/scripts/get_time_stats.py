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
import os
import sys

import numpy as np
import rosbag


def get_time_stats(bagfile, topic, time):
    with rosbag.Bag(bagfile, "r") as bag:
        times = []
        for _, msg, _ in bag.read_messages([topic]):
            times.append(getattr(msg, time))

        mean_time = np.mean(times)
        min_time = np.min(times)
        max_time = np.max(times)
        stddev_time = np.std(times)
        print(("Mean time: " + str(mean_time)))
        print(("Min time: " + str(min_time)))
        print(("Max time: " + str(max_time)))
        print(("Stddev time: " + str(stddev_time)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile")
    parser.add_argument("topic")
    parser.add_argument("time")
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    get_time_stats(args.bagfile, args.topic, args.time)
