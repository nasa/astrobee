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
Verify correctness of a ROS bag, as output by rosbag_fix_all.py. This
performs some simple checks that are not part of the standard "rosbag
check".
"""

from __future__ import print_function

import argparse
import json
import logging
import os
import sys
import traceback as tb

import rosbag


def get_message_counts(bag_path):
    try:
        with rosbag.Bag(bag_path, "r") as bag:
            types, topics = bag.get_type_and_topic_info()
            return {topic: info.message_count for topic, info in topics.items()}
    except:
        logging.warning("WARNING: Caught exception during message count check")
        logging.debug("  Exception was: %s", get_exception_text())
        return None


def message_counts_match(bag_path1, bag_path2):
    c1 = get_message_counts(bag_path1)
    c2 = get_message_counts(bag_path2)
    if c2 is None:
        logging.info(
            "INCONCLUSIVE: Message count comparison: The unfixed bag was too broken to support checking its message counts. (But the fixed bag is likely ok.)"
        )
        return True
    elif c1 == c2:
        num_topics = len(c1)
        num_messages = sum([mc for mc in c1.values()])
        logging.info("PASS: Input bags have identical message counts on each topic")
        logging.debug(
            "  Each file has a total of %d messages on %d topics",
            num_messages,
            num_topics,
        )
        return True
    else:
        logging.warning("FAIL: Input bags do not have identical message counts")
        logging.debug(
            "Bag message count info:\n%s",
            json.dumps({bag_path1: c1, bag_path2: c2}, indent=4, sort_keys=True),
        )
        return False


def get_exception_text():
    return tb.format_exception_only(*sys.exc_info()[:2])[0].strip()


def valid_deserialization(bag_path):
    """
    Force deserialization of messages, which can catch some binary
    corruption that is not detected by rosbag check.
    """
    try:
        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages():
                pass
    except:
        logging.warning("FAIL: Caught exception during message deserialization check")
        logging.debug("  Exception was: %s", get_exception_text())
        return False

    logging.info("PASS: Successfully deserialized messages")
    return True


def verify(fixed_path, orig_path, verbose=False, deserialize=False):
    result = True
    result &= message_counts_match(fixed_path, orig_path)
    if deserialize:
        result &= valid_deserialization(fixed_path)

    if result:
        logging.info("PASS: rosbag_verify: All tests passed")
        return 0
    else:
        logging.warning("FAIL: rosbag_verify: Some tests failed; check output above")
        return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-v",
        "--verbose",
        help="print more debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-d",
        "--deserialize",
        help="perform deserialization check (can be very slow on large bags)",
        default=False,
        action="store_true",
    )
    parser.add_argument("fixed_bag", help="Path to fixed bag to verify")
    parser.add_argument("orig_bag", help="Path to original bag to check against")

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    ret = verify(
        args.fixed_bag,
        args.orig_bag,
        verbose=args.verbose,
        deserialize=args.deserialize,
    )

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)

    sys.exit(ret)
