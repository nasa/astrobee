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
Generates an image showing all image space target detections for target 
detection files in a directory.
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np


def view_detections(directory):
    detection_files = [
        os.path.join(directory, filename)
        for filename in os.listdir(directory)
        if os.path.isfile(os.path.join(directory, filename))
        and filename.endswith(".txt")
    ]
    image_files = [
        os.path.join(directory, filename)
        for filename in os.listdir(directory)
        if os.path.isfile(os.path.join(directory, filename))
        and filename.endswith(".jpg")
    ]
    if len(detection_files) == 0 or len(image_files) == 0:
        print("No detection files or no images found in directory.")
        sys.exit()

    image = cv2.imread(image_files[0])
    height, width, channels = image.shape
    detection_image = np.zeros((height, width, 3), np.uint8)
    for detection_file in detection_files:
        with open(detection_file) as detection_csvfile:
            reader = csv.reader(detection_csvfile, delimiter=" ")
            for row in reader:
                cv2.circle(
                    detection_image,
                    (int(round(float(row[3]))), int(round(float(row[4])))),
                    1,
                    (0, 255, 0),
                    -1,
                )
    cv2.imwrite("detection_image.jpg", detection_image)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "-d",
        "--directory",
        default="./",
        help="Directory containing target detection files. Assumes at least one image is in same directory from which the image dimensions can be used for the generated image.",
    )
    args = parser.parse_args()
    if not os.path.isdir(args.directory):
        print("Directory " + args.directory + " does not exist.")
        sys.exit()
    view_detections(args.directory)
