#!/usr/bin/env python3
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
Partially recover bagfiles when corruption happens
"""


import argparse
import os
import re
import string
import subprocess
import sys

import yaml


def recover_bag(input_bag):

    # Execute the rosbag info command
    command = ["rosbag", "info", "-y", "-k", "topics", input_bag]
    result = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    # Check for errors
    if result.returncode != 0:
        print(f"Error running rosbag info: {result.stderr}")
        return []

    # Parse the YAML output
    topics_info = yaml.safe_load(result.stdout)

    # Process the topics into the desired format
    topic_names = [topic["topic"] for topic in topics_info]
    print(topic_names)

    topic_bags = []
    base_name, ext = os.path.splitext(input_bag)
    for topic in topic_names:
        out_bag_topic = base_name + "." + topic.replace("/", "_") + ext
        command = [
            "rosrun",
            "localization_node",
            "merge_bags",
            input_bag,
            "-output_bag",
            out_bag_topic,
            "-save_topics",
            topic,
        ]
        result = subprocess.run(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )

        # Check for errors
        if result.returncode != 0:
            print(f"Error running rosbag info: {result.stderr}")
            return []

        command = ["rosrun", "bag_processing", "rosbag_fix_all.py", out_bag_topic]
        result = subprocess.run(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        # Check for errors
        if result.returncode != 0:
            print(f"Error running rosbag info: {result.stderr}")
            return []

        topic_bags.append(base_name + "." + topic.replace("/", "_") + ".fix_all" + ext)

    out_bag = base_name + ".recovered" + ext
    command = (
        ["rosrun", "localization_node", "merge_bags"]
        + topic_bags
        + ["-output_bag", out_bag]
    )
    result = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    # Check for errors
    if result.returncode != 0:
        print(f"Error running rosbag info: {result.stderr}")
        return []

    command = ["rosrun", "bag_processing", "rosbag_fix_all.py", out_bag]
    result = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    # Check for errors
    if result.returncode != 0:
        print(f"Error running rosbag info: {result.stderr}")
        return []


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "input_bag",
        default="",
        help="Input bag name.",
    )

    args = parser.parse_args()

    recover_bag(args.input_bag)
