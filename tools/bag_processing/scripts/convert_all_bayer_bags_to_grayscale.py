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
Creates a new bagfile with grayscale images for each bag in the current directory using its bayer encoded images.
"""


import argparse

import convert_bayer_to_grayscale
import rosbag

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-b",
        "--bayer-image-topic",
        default="/hw/cam_nav_bayer",
        help="Bayer image topic name.",
    )
    parser.add_argument(
        "-g",
        "--gray-image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Output gray image topic.",
    )
    parser.add_argument(
        "-s",
        "--save-all-topics",
        dest="save_all_topics",
        action="store_true",
        help="Save all topics from input bagfile to output bagfile.",
    ) 
    args = parser.parse_args()
    for bag in glob.glob("*.bag"):
        convert_bayer_to_grayscale.convert_bayer_to_grayscale(bag, args.bayer_image_topic, args.gray_image_topic, args.save_all_topics)
