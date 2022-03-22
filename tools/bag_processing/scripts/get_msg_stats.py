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
Generates mean, min, max, and stddev stats for provided topic and its field
in a bagfile. 
"""

import argparse
import os
import sys

import numpy as np
import rosbag


def get_stats(bagfile, topic, field, use_size=False):
    with rosbag.Bag(bagfile, "r") as bag:
        vals = []
        for _, msg, _ in bag.read_messages([topic]):
            val = getattr(msg, field)
            if use_size:
                vals.append(len(val))
            else:
                vals.append(val)

        mean_val = np.mean(vals)
        min_val = np.min(vals)
        max_val = np.max(vals)
        stddev_val = np.std(vals)
        print(("Mean " + field + ": " + str(mean_val)))
        print(("Min " + field + ": " + str(min_val)))
        print(("Max " + field + ": " + str(max_val)))
        print(("Stddev " + field + ": " + str(stddev_val)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile containing desired topic.")
    parser.add_argument("topic", help="Topic in the bagfile to get stats for.")
    parser.add_argument(
        "field",
        help="Field of the message published on the provided topic to get stats for.",
    )
    parser.add_argument(
        "--use-size",
        "-s",
        default=False,
        action="store_true",
        help="Get stats for the size of the provided field.",
    )

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    get_stats(args.bagfile, args.topic, args.field, args.use_size)
