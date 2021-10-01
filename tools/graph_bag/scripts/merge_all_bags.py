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
import re
import string
import sys

import merge_bags
import rosbag

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--merged-bag", default="")
    parser.add_argument(
        "--only-loc-topics", dest="only_loc_topics", action="store_true"
    )
    args = parser.parse_args()

    # Find bagfiles with bag prefix in current directory, fail if none found
    bag_names = [
        (os.path.splitext(bag)[0]).rstrip(string.digits)
        for bag in os.listdir(".")
        if os.path.isfile(bag) and bag.endswith(".bag")
    ]
    # Remove duplicates
    bag_names = sorted(set(bag_names))
    if len(bag_names) == 0:
        print("No bag files found")
        sys.exit()
    else:
        print(("Found " + str(len(bag_names)) + " bag file prefixes."))

    for bag_name in bag_names:
        merge_bags.merge_bag(bag_name, args.merged_bag, args.only_loc_topics)
