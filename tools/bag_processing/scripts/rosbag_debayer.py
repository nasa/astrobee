#!/usr/bin/env python3
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
Converts bayer encoded images from the provided bagfile to grayscale and color images in a new bagfile.
"""

import argparse
import glob
import os
import pathlib
import sys
import time
from typing import Dict, List, Optional

import cv2
import rosbag
from cv_bridge import CvBridge, CvBridgeError

from bag_processing import pixel_utils as pu

# pylint: disable=c-extension-no-member  # pylint can't find cv2 members
# pylint: disable=line-too-long  # let black handle this


def get_correctors(
    inbag_paths: List[str],
    list_cam: List[str],
    bayer_image_topic: str,
    no_correct: bool,
    load_corrector: Optional[str],
    generate_corrector: str,
) -> Dict[str, Optional[pu.BadPixelCorrector]]:
    "Return loaded or generated correctors for cameras in `list_cam`."
    out: Dict[str, Optional[pu.BadPixelCorrector]] = {}
    if no_correct:
        return out
    corrector_class, accum_class = pu.BadPixelCorrector.get_classes(generate_corrector)
    for cam in list_cam:
        if load_corrector is not None:
            # Load pre-configured BadPixelCorrector
            corrector_path = pathlib.Path(load_corrector.format(cam=cam))
            print(f"Loading corrector for {cam} from {corrector_path}")
            corrector: Optional[pu.BadPixelCorrector] = pu.BadPixelCorrector.load(
                corrector_path
            )
        else:
            # Generate BadPixelCorrector of specified type
            t0 = time.time()
            print(
                f"Generating {generate_corrector} for {cam} by analyzing images... ",
                end="",
            )
            sys.stdout.flush()
            topic = bayer_image_topic.format(cam=cam)
            images = pu.ImageSourceBagPaths(inbag_paths, topic=topic)
            stats = accum_class.get_image_stats_parallel(images)
            if stats is None:
                corrector = None
            else:
                note = f"bags={inbag_paths} topic='{topic}'"
                corrector = corrector_class.from_image_stats(stats, note)
            t1 = time.time()
            print(f"done in {t1 - t0:.1f}s")
            if corrector is None:
                print(f"Can't generate corrector for {cam}, not enough images")
            else:
                corrector.save(
                    pathlib.Path(f"{corrector.__class__.__name__}_{cam}_cam.json")
                )
        out[cam] = corrector
    return out


def convert_bayer(
    bagfile,
    output_bag_name,
    list_cam,
    bayer_image_topic,
    gray_image_topic,
    color_image_topic,
    save_all_topics=False,
    correctors: Optional[Dict[str, Optional[pu.BadPixelCorrector]]] = None,
):
    bridge = CvBridge()
    topics = dict((bayer_image_topic.format(cam=cam), cam) for cam in list_cam)
    output_bag = rosbag.Bag(output_bag_name, "w")
    topics_bag = [] if save_all_topics else topics

    with rosbag.Bag(bagfile, "r") as bag:
        for topic, msg, t in bag.read_messages(topics_bag):
            if topic in topics:
                try:
                    bayer_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
                except CvBridgeError as exc:
                    print(exc)
                    continue

                cam = topics[topic]
                if correctors:
                    corrector = correctors[cam]
                    if corrector is not None:
                        bayer_image = corrector(bayer_image)

                # Check if we should save greyscale image
                if gray_image_topic != "":
                    # Technically this color conversion should be using cv2.COLOR_BAYER_GB2GRAY to
                    # match the true Bayer convention of the Astrobee NavCam/DockCam. Using
                    # cv2.COLOR_BAYER_GR2GRAY swaps the R/B channels of the Bayer pattern.  A color
                    # channel swap has a subtle effect on grayscale output because different color
                    # channels have different weighting factors when outputting luminance calibrated
                    # for human perception.  However, for the purposes of this script, the key
                    # requirement is to exactly replicate the (similarly erroneous) onboard debayer
                    # conversion performed in the FSW is_camera ROS node, so localization features
                    # will be the same regardless of which tool is used to do the conversion.
                    # https://github.com/nasa/astrobee/blob/develop/hardware/is_camera/src/camera.cc#L522
                    # https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
                    gray_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_GR2GRAY)
                    gray_image_msg = bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
                    gray_image_msg.header = msg.header
                    output_bag.write(
                        gray_image_topic.format(cam=cam),
                        gray_image_msg,
                        t,
                    )
                # Check if we should save color image
                if color_image_topic != "":
                    # Here we are using the correct Bayer convention because we want the color image
                    # to look right and we have no need to replicate legacy onboard debayering.
                    color_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_GB2BGR)
                    color_image_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                    color_image_msg.header = msg.header
                    output_bag.write(
                        color_image_topic.format(cam=cam),
                        color_image_msg,
                        t,
                    )
            if save_all_topics:
                output_bag.write(topic, msg, t)
    output_bag.close()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "inbag",
        nargs="+",
        help="List of bags to convert. If none provided, all bags in the current directory are used.",
    )
    parser.add_argument(
        "-o", "--output", help="path for output bag", default="debayer_{inbag}"
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
        default="/hw/cam_{cam}_bayer",
        help="Bayer image topic name.",
    )
    parser.add_argument(
        "--no-gray",
        dest="disable_gray",
        action="store_true",
        help="Disable grayscale conversion.",
    )
    parser.add_argument(
        "-g",
        "--gray-image-topic",
        default="/mgt/img_sampler/{cam}_cam/image_record",
        help="Output gray image topic.",
    )
    parser.add_argument(
        "--no-color",
        dest="disable_color",
        action="store_true",
        help="Disable color conversion.",
    )
    parser.add_argument(
        "-c",
        "--color-image-topic",
        default="/hw/cam_{cam}/image_color",
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
        "--no-correct",
        action="store_true",
        default=False,
        help="Skip bad pixel correction",
    )
    parser.add_argument(
        "--load-corrector",
        default=None,
        help="Load pre-configured bad pixel correctors using path template with '{cam}', e.g. 'corrector_{cam}_cam.json'",
    )
    parser.add_argument(
        "--generate-corrector",
        choices=("BiasCorrector", "NeighborMeanCorrector"),
        default="BiasCorrector",
        help="Generate bad pixel corrector of specified type, if needed",
    )
    parser.add_argument(
        "-n",
        dest="do_nothing",
        action="store_true",
        help="Option to not debayer anything and write output",
    )
    args = parser.parse_args()

    if args.disable_gray:
        args.gray_image_topic = ""
    if args.disable_color:
        args.color_image_topic = ""

    inbag_paths = args.inbag if args.inbag is not None else glob.glob("*.bag")

    if not args.do_nothing:
        correctors = get_correctors(
            inbag_paths=inbag_paths,
            list_cam=args.list_cam,
            bayer_image_topic=args.bayer_image_topic,
            no_correct=args.no_correct,
            load_corrector=args.load_corrector,
            generate_corrector=args.generate_corrector,
        )

    for inbag_path in inbag_paths:
        # Check if input bag exists
        if not os.path.isfile(inbag_path):
            print(("Bag file " + inbag_path + " does not exist."))
            sys.exit(1)
        output_bag_name = args.output.format(inbag=inbag_path)

        # Check if output bag already exists
        if os.path.exists(output_bag_name):
            parser.error("not replacing existing file %s" % output_bag_name)
        if not args.do_nothing:
            # Conver bayer topic to black/white and color
            convert_bayer(
                inbag_path,
                output_bag_name,
                args.list_cam,
                args.bayer_image_topic,
                args.gray_image_topic,
                args.color_image_topic,
                args.save_all_topics,
                correctors,
            )
        else:
            os.rename(inbag_path, output_bag_name)


if __name__ == "__main__":
    main()
