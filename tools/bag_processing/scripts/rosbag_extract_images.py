#!/usr/bin/env python3
# Copyright (c) 2024, United States Government, as represented by the
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
Read images on specified topic from bag and write them to specified output folder in PNG format.
Output images will be named according to ROS message timestamp.
"""

import argparse
import glob
import itertools
import pathlib
from typing import Iterable, List, Optional, Tuple

import cv2
import numpy as np
import rosbag
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge

# pylint: disable=c-extension-no-member  # because pylint can't find cv2 members

# White balance and contrast-enhance RGB image to improve appearance, assuming it was taken by NavCam
# on ISS. The contrast enhancement in particular might need to be tuned depending on exposure settings.
WHITE_BALANCE_LEVELS = np.array([1.0000, 0.5250, 0.6818])
CONTRAST_ENHANCE = 1.8


def message_source(
    bagfiles: List[str], topic: str
) -> Iterable[Tuple[sensor_msgs.msg.Image, rospy.rostime.Time]]:
    "Return a generator that reads `bagfiles` on `topic` and yields (message, timestamp) pairs."
    for bagfile in bagfiles:
        with rosbag.Bag(bagfile, "r") as bag:
            print(f"Reading images from: {bagfile}")
            for _, msg, t in bag.read_messages([topic]):
                yield msg, t


def rosbag_extract_images(
    bagfiles: List[str],
    output_dir: pathlib.Path,
    topic: str,
    head: Optional[int],
    ratio: int,
):
    "Main driver for extracting images."
    bridge = CvBridge()
    messages = message_source(bagfiles, topic)
    if not (head is None and ratio == 1):
        msg = f"Extracting 1 out of every {ratio} images"
        if head is not None:
            msg += f" for the first {head} images"
        print(msg)
        messages = itertools.islice(messages, 0, head, ratio)

    count = 0
    for msg, t in messages:
        image = bridge.imgmsg_to_cv2(msg, msg.encoding)  # type: ignore  # mypy doesn't have msg definition
        if msg.encoding == "bgr8":  # type: ignore
            adjust = WHITE_BALANCE_LEVELS * CONTRAST_ENHANCE
            image = np.clip(image * adjust[np.newaxis, np.newaxis, :], 0, 255).astype(
                np.uint8
            )
        cv2.imwrite(str(output_dir / str(t)) + ".png", image)
        count += 1

    print(f"Wrote {count} images to: {output_dir}")


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    "Formatter with multiple mixins."


def main():
    "Parse command-line args and call rosbag_extract_images()"
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "inbag",
        nargs="+",
        default=[],
        help="bags to extract images from",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="folder for writing output images",
        default="rosbag_extract_images_output",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default="/hw/cam_nav_bayer",
        help="image topic name.",
    )
    parser.add_argument(
        "--head",
        type=int,
        default=None,
        help="process first HEAD images only",
    )
    parser.add_argument(
        "--ratio",
        type=int,
        default=1,
        help="process 1 out of every RATIO images",
    )
    args = parser.parse_args()

    output = pathlib.Path(args.output)
    if output.exists():
        parser.error("not overwriting --output folder {output}")
    output.mkdir(parents=True)

    inbag_paths = args.inbag if args.inbag else glob.glob("*.bag")

    rosbag_extract_images(
        inbag_paths,
        output_dir=output,
        topic=args.topic,
        head=args.head,
        ratio=args.ratio,
    )


if __name__ == "__main__":
    main()
