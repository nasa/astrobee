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
Given two bags containing the message topics output by
pico_split_extended.py, aligns matching messages by publish timestamp
and checks their contents are the same. For "points" messages, the check
includes a (very strict) error tolerance to accommodate round-off
error. For "amplitude" messages, the check is for exact integer
equality.
"""

import argparse
import logging

import numpy as np

import pico_utils as pico


def check_points_msgs(
    orig_bag_path, test_bag_path, fast=False, verbose=False, cam=pico.DEFAULT_CAM
):
    pts_topic = pico.get_pts_topic(cam)
    orig_msgs = pico.get_msg_tuples(orig_bag_path, pts_topic)
    test_msgs = pico.get_msg_tuples(test_bag_path, pts_topic)

    # In raw robot bags studied so far, the extended message is reliably
    # published <100 ms after the points message. The points messages in
    # orig_bag are timestamped like points messages.  The points messages in
    # test_bag will have timestamps the same as the extended messages they are
    # derived from.  We set dt_limit to reflect this.
    pair_stream = pico.get_matched_msg_pairs(
        orig_msgs,
        test_msgs,
        dt_limit=pico.EXT_MINUS_PTS_DT_LIMIT,
    )
    if fast:
        pair_stream = pico.fast_filter(pair_stream)

    success = True
    err_max_log = []
    err_rms_log = []

    k_logger = pico.PrintEveryK(100)
    for i, (orig_msg, test_msg) in enumerate(pair_stream):
        msg_success, err_max, err_rms = pico.check_points_msg_pair(
            orig_msg, test_msg, i, verbose
        )
        success &= msg_success
        err_max_log.append(err_max)
        err_rms_log.append(err_rms)

        k_logger.info("checked %5d points messages", k_logger.count + 1)

    logging.info("Summary over all %d points messages:", k_logger.count)
    logging.info(" Error max (um): %.3f", np.max(err_max_log) * 1e6)
    logging.info(" Mean error RMS (um): %.3f", np.mean(err_rms_log) * 1e6)

    return success


def check_amp_msgs(
    orig_bag_path, test_bag_path, fast=False, verbose=False, cam=pico.DEFAULT_CAM
):
    amp_topic = pico.get_amp_topic(cam)
    orig_msgs = pico.get_msg_tuples(orig_bag_path, amp_topic)
    test_msgs = pico.get_msg_tuples(test_bag_path, amp_topic)

    # In the original bag, we should be guaranteed that the amp message comes
    # after the extended message, because it is derived from the extended
    # message by the pico_proxy nodelet. The amp messages in orig_bag should be
    # timestamped like amp messages, but the timestamps in test_bag inherit
    # their timestamps from the extended messages. Therefore dt should be negative.
    pair_stream = pico.get_matched_msg_pairs(
        orig_msgs,
        test_msgs,
        dt_limit=(-0.2, 0),
    )
    if fast:
        pair_stream = pico.fast_filter(pair_stream)

    success = True

    k_logger = pico.PrintEveryK(100)
    for i, (orig_msg, test_msg) in enumerate(pair_stream):
        success &= pico.check_amp_msg_pair(orig_msg, test_msg, i, verbose)

        k_logger.info("checked %5d amplitude_int messages", k_logger.count + 1)

    return success


def pico_check_split_extended(
    orig_bag_path, test_bag_path, fast=False, verbose=False, cam=pico.DEFAULT_CAM
):
    success = True
    success &= check_points_msgs(
        orig_bag_path, test_bag_path, fast=fast, verbose=verbose, cam=cam
    )
    success &= check_amp_msgs(
        orig_bag_path, test_bag_path, fast=fast, verbose=verbose, cam=cam
    )

    if success:
        print("OK - All checks passed.")
    else:
        print("FAILED - Some checks failed. See previous messages.")


def main():
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
    parser.add_argument(
        "-f",
        "--fast",
        help="speed testing up by only processing part of the bags",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-c",
        "--cam",
        help="specify camera for rosbag topic filtering [%(default)s]",
        nargs="?",
        choices=pico.CAM_CHOICES,
        default=pico.DEFAULT_CAM,
    )
    parser.add_argument(
        "orig_bag", help="input bag containing original points messages"
    )
    parser.add_argument(
        "test_bag",
        help="input bag containing points messages to check against original messages",
    )

    args = parser.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    pico_check_split_extended(
        args.orig_bag,
        args.test_bag,
        fast=args.fast,
        verbose=args.verbose,
        cam=args.cam,
    )

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)


if __name__ == "__main__":
    main()
