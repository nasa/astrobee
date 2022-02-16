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
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile")
    parser.add_argument("topic")
    parser.set_defaults(adaptive=True)
    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()
    
    output_bag_name = os.path.splitext(args.bagfile)[0] + "_mono.bag" 
    output_bag = rosbag.Bag(output_bag_name, 'w')
    bridge = CvBridge()

    with rosbag.Bag(args.bagfile, "r") as bag:
        for _, msg, _ in bag.read_messages([args.topic]):
            try:
                image = bridge.imgmsg_to_cv2(msg, "mono8")
            except CvBridgeError, e:
                print(e)
            mono_image = cv2.cvtColor(image, cv2.COLOR_BAYER_GR2GRAY)
            mono_image_msg = bridge.cv2_to_imgmsg(mono_image, encoding="mono8")
            mono_image_msg.header = msg.header
            output_bag.write("mono_image", mono_image_msg, msg.header.stamp)
    
    output_bag.close()
