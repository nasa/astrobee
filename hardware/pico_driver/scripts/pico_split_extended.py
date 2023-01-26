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
Given a bag containing /hw/depth_*/extended messages and an xyz coefficients
file output by pico_write_xyz_coeff.py, output a bag of /hw/depth_*/points
messages.
"""

import argparse
import logging
import os
import shutil

import numpy as np
import pico_utils as pico
import rosbag


class PrintEveryK:
    def __init__(self, k):
        self.k = k
        self.count = 0

    def debug(self, *args, **kwargs):
        self.count += 1
        if (self.count % self.k) == 0:
            logging.debug(*args, **kwargs)


def pico_xyz_from_extended(
    in_bag_path,
    in_npy_path,
    out_bag_path,
    fast=False,
    verbose=False,
    cam=pico.DEFAULT_CAM,
    save_all_topics=False,
):
    with open(in_npy_path, "rb") as in_npy:
        xyz_coeff = np.load(in_npy)

    k_logger = pico.PrintEveryK(100)
    ext_topic = pico.get_ext_topic(cam)
    pts_topic = pico.get_pts_topic(cam)
    amp_topic = pico.get_amp_topic(cam)
    topics_bag = [] if save_all_topics else ext_topic

    with rosbag.Bag(in_bag_path, "r") as bag, rosbag.Bag(out_bag_path, "w") as out_bag:
        # for ext_msg in pico.get_msgs(in_bag_path, ext_topic, fast):
        for topic, msg, t in bag.read_messages(topics_bag):
            if topic == ext_topic:
                distance, amplitude, intensity, noise = pico.split_extended(msg)

                xyz = pico.xyz_from_distance(distance, xyz_coeff)
                pts_msg = pico.make_points_msg(msg.header, xyz)
                out_bag.write(pts_topic, pts_msg, pts_msg.header.stamp)

                amp_msg = pico.make_amp_msg(msg.header, amplitude)
                out_bag.write(amp_topic, amp_msg, amp_msg.header.stamp)

                k_logger.info("split %5d extended messages", k_logger.count + 1)

            if save_all_topics:
                out_bag.write(topic, msg, t)
    logging.info("wrote split messages to %s", out_bag_path)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "inbag", nargs="+", help="Input bag containing extended messages"
    )
    parser.add_argument("--in_npy", help="Input xyz coefficients file", default="")
    parser.add_argument(
        "-o",
        "--output",
        help="path for output bag",
        default="{inbag}.split_extended.bag",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="print more verbose debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-f",
        "--fast",
        help="speed testing up by only processing part of the bag",
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
        "-s",
        "--save-all-topics",
        dest="save_all_topics",
        action="store_true",
        help="Save all topics from input bagfile to output bagfile.",
    )
    parser.add_argument(
        "-n",
        dest="do_nothing",
        action="store_true",
        help="Option to not debayer anything and write output",
    )

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    for inbag_path in args.inbag:
        # Check if input bag exists
        if not os.path.isfile(inbag_path):
            print(("Bag file " + inbag_path + " does not exist."))
            sys.exit()
        output_bag_name = args.output.format(inbag=inbag_path)

        # Check if output bag already exists
        if os.path.exists(output_bag_name):
            parser.error("not replacing existing file %s" % output_bag_name)

        if not args.do_nothing:
            # Split extended message
            pico_xyz_from_extended(
                inbag_path,
                args.in_npy,
                output_bag_name,
                args.fast,
                args.verbose,
                args.cam,
                args.save_all_topics,
            )
        else:
            os.rename(inbag_path, output_bag_name)

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)


if __name__ == "__main__":
    main()
