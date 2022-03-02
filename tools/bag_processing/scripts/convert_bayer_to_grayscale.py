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
Converts bayer encoded images from the provided bagfile to grayscale images in a new bagfile.
"""

import argparse
import os
import sys

import cv2
import rosbag
import rospy
import utilities
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def convert_bayer_to_grayscale(bagfile, bayer_image_topic, gray_image_topic, save_all_topics = False):
    bridge = CvBridge()
    topics = None if save_all_topics else [bayer_image_topic]
    output_bag_name = os.path.splitext(bagfile)[0] + "_gray.bag"
    output_bag = rosbag.Bag(output_bag_name, "w")

    with rosbag.Bag(bagfile, "r") as bag:
        for topic, msg, t in bag.read_messages(topics):
            if topic == bayer_image_topic:
                try:
                    image = bridge.imgmsg_to_cv2(msg, "mono8")
                except (CvBridgeError) as e:
                    print(e)
                gray_image = cv2.cvtColor(image, cv2.COLOR_BAYER_GR2GRAY)
                gray_image_msg = bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
                gray_image_msg.header = msg.header
                output_bag.write(gray_image_topic, gray_image_msg, t)
            else:
                output_bag.write(topic, msg, t)
    output_bag.close()
 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile with bayer images.")
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
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    convert_bayer_to_grayscale(args.bagfile, args.bayer_image_topic, args.gray_image_topic, args.save_all_topics)
