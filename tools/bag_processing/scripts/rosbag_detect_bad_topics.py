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
Detect bad topics that the rosbag API can't handle. (The main problem
so far has been rosjava messages that have incomplete message definition
dependency information that causes rosbag to raise an exception when
it tries to parse the message.)
"""

from __future__ import print_function

import argparse
import logging

import rosbag


def detect_bad_topic(inbag_path):
    with rosbag.Bag(inbag_path, "r") as inbag:
        logging.info("== phase 1: extract topic list on first pass ")
        all_topics = inbag.get_type_and_topic_info()[1].keys()
        logging.info("  %s topics found", len(all_topics))
        maybe_bad_topics = set(all_topics)

        logging.info(
            "\n== phase 2: quickly identify as many good topics as possible in one pass"
        )
        try:
            for topic, msg, t in inbag.read_messages(maybe_bad_topics, raw=True):
                if topic in maybe_bad_topics:
                    logging.info("good: %s", topic)
                maybe_bad_topics.remove(topic)
        except:
            pass
        logging.info("%s remaining topics:", len(maybe_bad_topics))
        logging.info("  %s\n", "\n  ".join(sorted(maybe_bad_topics)))

        logging.info(
            "\n== phase 3: check remaining topics one at a time, one pass for each topic"
        )
        bad_topics = []
        for check_topic in sorted(maybe_bad_topics):
            logging.info("checking: %s", check_topic)
            try:
                for topic, msg, t in inbag.read_messages([check_topic], raw=True):
                    logging.info("  good")
                    break
            except:
                logging.info("  bad")
                bad_topics.append(check_topic)

        logging.info("\n== check complete, final bad topics ==")
        print("\n".join(bad_topics))
        logging.info("")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="Print debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument("inbag", help="Input bag")

    args = parser.parse_args()
    level = logging.INFO if args.verbose else logging.WARN
    logging.basicConfig(level=level, format="%(message)s")
    detect_bad_topic(args.inbag)
