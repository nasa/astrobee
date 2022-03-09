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
Creates a new bagfile with grayscale images for the provided bags using their bayer encoded images.
If no bags are provided, runs conversion for each bag in the current directory.
"""


import argparse
import glob

import convert_bayer_bag
import rosbag

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-l",
        "--list-cam",
        default=["nav"],
        help="Cameras to be converted, default nav, can add dock.",
        nargs="+",
        type=str,
    )
    parser.add_argument(
        "-b",
        "--bayer-image-topic",
        default="/hw/cam_nav_bayer",
        help="Bayer image topic name.",
    )
    parser.add_argument(
        "-g",
        dest="disable_gray",
        action="store_false",
        help="Disable grayscale conversion.",
    )
    parser.add_argument(
        "--gray-image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Output gray image topic.",
    )
    parser.add_argument(
        "-c",
        dest="disable_color",
        action="store_false",
        help="Disable color conversion.",
    )
    parser.add_argument(
        "--color-image-topic",
        default="/hw/cam_nav/image_color",
        help="Output color image topic.",
    )
    parser.add_argument(
        "-s",
        "--save-all-topics",
        dest="save_all_topics",
        action="store_true",
        help="Save all topics from input bagfile to output bagfile.",
    )
    parser.add_argument(
        "-k",
        "--keep-bayer",
        dest="keep_bayer_topic",
        action="store_true",
        help="Save bayer topic alongside converted image on the new bagfile",
    )
    parser.add_argument(
        "--bags",
        nargs="*",
        help="List of bags to convert. If none provided, all bags in the current directory are used.",
    )

    if args.disable_gray:
        args.gray_image_topic = ""
    if args.disable_color:
        args.color_image_topic = ""

    args = parser.parse_args()
    bags = args.bags if args.bags is not None else glob.glob("*.bag")
    for bag in bags:
        convert_bayer_bag.convert_bayer(
            bag,
            args.list_cam,
            args.bayer_image_topic,
            args.gray_image_topic,
            args.color_image_topic,
            args.save_all_topics,
            args.keep_bayer_topic,
        )
