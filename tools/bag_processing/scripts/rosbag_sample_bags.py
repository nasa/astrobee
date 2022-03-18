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
Write an overview bag that contains one message on each unique topic, collected
from the specified input bags. The resulting overview bag allows for quick
backward compatibility testing with the latest ROS packages.
"""

from __future__ import print_function

import argparse
import collections
import fnmatch
import json
import logging
import os

import genpy
import rosbag
import roslib

import rosbag_rewrite_types as rrt


def sample_bags(inbag_paths, outbag_path, rules_files, verbose=False):
    if os.path.exists(outbag_path):
        logging.critical("not overwriting existing file %s", outbag_path)
        return

    fix_topic_patterns, _ = rrt.get_rules(rules_files)

    sampled = set()
    with rosbag.Bag(outbag_path, "w") as outbag:
        for inbag_path in inbag_paths:
            if verbose:
                logging.info("scanning bag: %s", inbag_path)
            with rosbag.Bag(inbag_path, "r") as inbag:
                rrt.fix_message_definitions(inbag, fix_topic_patterns)
                for topic, msg, t in inbag.read_messages(raw=True):
                    if topic in sampled:
                        continue
                    sampled.add(topic)
                    if verbose:
                        logging.info("found new topic: %s", topic)
                    outbag.write(topic, msg, t, raw=True)
    logging.info("found %s topics in %s bags", len(sampled), len(inbag_paths))
    logging.info("wrote one sample message on each topic to %s", outbag_path)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="print more debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="path for output bag",
        default="sample_bags.bag",
    )
    parser.add_argument(
        "-r",
        "--rules",
        nargs="?",
        help="path to input rules file (specify multiple times for multiple files)",
        default=[],
        action="append",
    )
    parser.add_argument("inbag", nargs="+", help="input bag")

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    sample_bags(args.inbag, args.output, args.rules, verbose=args.verbose)

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)
