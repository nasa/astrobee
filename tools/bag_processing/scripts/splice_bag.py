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
Splices a bagfile at selected timestamps to create multiple smaller bagfiles which when combined 
span the original bagfile. Uses wasd controls to iterate through the images.
Iterate through the images one at a time using the d and a keys.
Iterate more quickly (skipping 10 images at a time) using the w and s keys.
Create a splice point using the current image's timestamp using the space bar. 
Undo adding last splice point using the u key.
Print current splice points and intervals using the p key.
Generate spliced bags using all selected splice points by pressing the enter key at
any time. To exit without splicing, use the escape or q keys.
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


def print_info(splice_timestamps, bag_start_time, bag_end_time):
    print(str(len(splice_timestamps)) + " splice timestamps selected.")
    print("Splicing timestamps: " + str(splice_timestamps))
    print("Splicing intervals: ")
    starting_splice_percent = 0
    for i in range(len(splice_timestamps) + 1):
        ending_splice_percent = (
            (splice_timestamps[i] - bag_start_time)
            / float(bag_end_time - bag_start_time)
            if i < len(splice_timestamps)
            else 1
        )
        print(
            str(i)
            + ": "
            + "{0:.2f}".format(starting_splice_percent * 100)
            + "% to "
            + "{0:.2f}".format(ending_splice_percent * 100)
            + "%"
        )
        starting_splice_percent = ending_splice_percent


def sort_and_remove_repeats(splice_timestamps):
    # Remove repeats
    splice_timestamps = list(set(splice_timestamps))
    splice_timestamps.sort()
    return splice_timestamps


def show_image_with_message(
    image, window, title, message, origin=(50, 450), font_size=4, timeout=750
):
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    splice_image = cv2.putText(
        color_image,
        message,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        font_size,
        (0, 0, 255),
        10,
    )
    cv2.imshow(window, splice_image)
    cv2.setWindowTitle(window, title)
    key = cv2.waitKey(timeout)


def make_spliced_bag(bagfile, start_time, end_time, index):
    spliced_bagfile = os.path.splitext(bagfile)[0] + "_" + str(index) + ".bag"
    with rosbag.Bag(bagfile, "r") as bag:
        with rosbag.Bag(spliced_bagfile, "w") as spliced_bag:
            for topic, msg, t in bag.read_messages(
                None, rospy.Time(start_time), rospy.Time(end_time)
            ):
                spliced_bag.write(topic, msg, t)


def splice_bag(bagfile, splice_timestamps):
    if not splice_timestamps:
        print("No timestamps provided, not splicing.")
        return
    start_and_end_times = []
    with rosbag.Bag(bagfile, "r") as bag:
        # Subtract extra time from bag start time to ensure first message included
        start_time = bag.get_start_time() - 1.0
        for i in range(len(splice_timestamps) + 1):
            # Add extra time to end time to ensure final message included
            end_time = (
                splice_timestamps[i]
                if i < len(splice_timestamps)
                else bag.get_end_time() + 1.0
            )
            make_spliced_bag(bagfile, start_time, end_time, i)
            # Add slight extra time so new start time doesn't overlap with previous end time
            start_time = end_time + 1e-4


def select_splice_timestamps_and_splice_bag(bagfile, image_topic):
    splice_timestamps = []
    bridge = CvBridge()
    with rosbag.Bag(bagfile, "r") as bag:
        msg_tuples = []
        print("Reading msgs...")
        for topic, msg, t in bag.read_messages([image_topic]):
            msg_tuples.append((msg, t))
        if not msg_tuples:
            print("No messages found for topic: " + image_topic)
            sys.exit()
        i = 0
        num_msgs = len(msg_tuples)
        window = "image"
        cv2.namedWindow(window)
        cv2.moveWindow(window, 40, 30)
        print_string = True
        while True:
            msg = (msg_tuples[i])[0]
            timestamp = ((msg_tuples[i])[1]).to_sec()
            progress = i / float(num_msgs) * 100
            msg_info_string = (
                "{0:.2f}".format(progress)
                + "%, Image "
                + str(i)
                + "/"
                + str(num_msgs)
                + ", t: "
                + str(timestamp)
            )
            if print_string:
                print(msg_info_string)
                print_string = False
            try:
                image = bridge.imgmsg_to_cv2(msg, msg.encoding)
            except (CvBridgeError) as e:
                print(e)
            cv2.imshow(window, image)
            cv2.setWindowTitle(window, msg_info_string)
            key = cv2.waitKey(0)
            # Have to use wasd instead of arrow keys due to opencv/QT bug where
            # arrow presses aren't registered when the user clicks of the screen then back to the screen
            if key == ord("d"):
                i += 1
                print_string = True
            elif key == ord("a"):
                i -= 1
                print_string = True
            if key == ord("w"):
                i += 10
                print_string = True
            elif key == ord("s"):
                i -= 10
                print_string = True

            elif key == ord("q") or key == 27:  # Escape key
                print("Manually closing program, no splice operation applied.")
                exit(0)
            elif key == ord("p"):
                print_info(splice_timestamps, bag.get_start_time(), bag.get_end_time())
                show_image_with_message(
                    image,
                    window,
                    "Printed splice intervals",
                    "Printed splice intervals",
                    (130, 450),
                    3,
                )
            elif key == 32:  # Space bar
                print("Splice timestamp selected, t: " + str(timestamp))
                splice_timestamps.append(timestamp)
                splice_timestamps = sort_and_remove_repeats(splice_timestamps)
                print_info(splice_timestamps, bag.get_start_time(), bag.get_end_time())
                show_image_with_message(
                    image,
                    window,
                    "Splice t selected! " + msg_info_string,
                    "Added splice point",
                )
            elif key == ord("u"):
                if not splice_timestamps:
                    message = "No splice timestamps added."
                    print(message)
                    show_image_with_message(
                        image, window, message, message, (80, 450), 2.4
                    )
                else:
                    print(
                        "Removed last added splice point at timestamp: "
                        + str(timestamp)
                    )
                    splice_timestamps.pop()
                    print_info(
                        splice_timestamps, bag.get_start_time(), bag.get_end_time()
                    )
                    show_image_with_message(
                        image,
                        window,
                        "Removed last splice point at timestamp " + str(timestamp),
                        "Removed last splice point",
                        (40, 450),
                        3,
                    )

            elif key == 13:  # Enter key
                if not splice_timestamps:
                    message = "No splice timestamps added."
                    print(message)
                    show_image_with_message(
                        image, window, message, message, (80, 450), 2.4
                    )
                else:
                    print("Splicing bag using selected timestamps.")
                    print_info(
                        splice_timestamps, bag.get_start_time(), bag.get_end_time()
                    )
                    show_image_with_message(
                        image,
                        window,
                        "Creating spliced bags",
                        "Creating spliced bags",
                        (20, 450),
                        3.5,
                    )
                    splice_bag(bagfile, splice_timestamps)
                    return

            if i < 0:
                i = 0
            if i >= num_msgs:
                i = num_msgs - 1
                show_image_with_message(
                    image, window, "End of bag", "End of bag", (350, 450), 3
                )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/hw/cam_nav",
        help="Image topic name.",
    )

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    select_splice_timestamps_and_splice_bag(args.bagfile, args.image_topic)
