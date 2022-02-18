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
Applies histogram equalization to images in the provided bagfile and saves the results to a new bagfile.
Either uses normal histogram equalization or CLAHE adaptive equalization.
"""

import argparse
import os
import sys

import rosbag
import rospy
import utilities

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("bagfile", help="Input bagfile with grayscale images.")
    parser.add_argument("-t",
                        "--topic",
                        default="/mgt/img_sampler/nav_cam/image_record",
                        help="Image topic in bagfile.")
    parser.add_argument(
        '-n',
        '--non-adaptive',
        dest='adaptive',
        action='store_false',
        help=
        "Changes equalization approach from adaptive (CLAHE) to non-adaptive (normal histrogram equalization). Default approach is CLAHE."
    )
    parser.add_argument(
        "-s",
        "--save-all-topics",
        dest="save_all_topics",
        action="store_true",
        help="Save all topics from input bagfile to output bagfile.")
    parser.set_defaults(adaptive=True)
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    output_bag_name = os.path.splitext(
        args.bagfile)[0] + "_clahe.bag" if args.adaptive else os.path.splitext(
            args.bagfile)[0] + "_hist_equalized.bag"
    output_bag = rosbag.Bag(output_bag_name, 'w')
    bridge = CvBridge()
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    topics = None if args.save_all_topics else [args.topic]

    with rosbag.Bag(args.bagfile, "r") as bag:
        for topic, msg, t in bag.read_messages(topics):
            if topic == args.topic:
                try:
                    image = bridge.imgmsg_to_cv2(msg, "mono8")
                except (CvBridgeError) as e:
                    print(e)
                equalized_image = clahe.apply(
                    image) if args.adaptive else cv2.equalizeHist(image)
                equalized_image_msg = bridge.cv2_to_imgmsg(equalized_image,
                                                           encoding="mono8")
                equalized_image_msg.header = msg.header
                output_bag.write(args.topic, equalized_image_msg, t)
            else:
                output_bag.write(topic, msg, t)

    output_bag.close()
