#!/usr/bin/python3
#
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
Initializes the localizer with a defined pose by sending a ff_msgs VisualLandmarks message 
with the pose value.
"""

import argparse
import os
import sys

import rospy
import std_msgs.msg
from ff_msgs.msg import VisualLandmark, VisualLandmarks


def publish_pose(pose):
    # Init publisher
    pub = rospy.Publisher("/loc/ml/features", VisualLandmarks, queue_size=1)
    rospy.init_node("PosePublisher")
    # Sleep so node can subscribe to /clock topic and produce a valid header time
    rospy.sleep(5)

    msg = VisualLandmarks()
    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.camera_id = 0
    # Set pose values
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    # Set empty landmark features, make sure to have > 5 so msg is considered valid
    for i in range(0, 5):
        landmark = VisualLandmark()
        landmark.x = 0
        landmark.y = 0
        landmark.z = 0
        landmark.u = 0
        landmark.v = 0
        msg.landmarks.append(landmark)

    pub.publish(msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "pose",
        nargs="+",
        type=float,
        help="Start pose in the format: x y z qx qy qz qw. If --position-only used, only x y z are required.",
    )
    parser.add_argument(
        "-p",
        "--position-only",
        dest="initialize_position_only",
        action="store_true",
        help="Initialize a pose using only the position, the orientation will be set to identity.",
    )
    args = parser.parse_args()
    if not args.initialize_position_only and len(args.pose) != 7:
        print("Pose requires 7 fields.")
        sys.exit()
    if args.initialize_position_only and len(args.pose) != 3:
        print("Position only pose requires 3 fields.")
        sys.exit()
    if args.initialize_position_only:
        args.pose.extend([0, 0, 0, 1])
    publish_pose(args.pose)
