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
import csv
import os
import sys

import cv2
import numpy as np


def view_detections(directory):
  detection_files = [
    directory + filename
    for filename in os.listdir(directory)
    if os.path.isfile(directory + filename) and filename.endswith(".txt")
  ]
  image_files = [
    directory + filename
    for filename in os.listdir(directory)
    if os.path.isfile(directory + filename) and filename.endswith(".jpg")
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
        cv2.circle(detection_image, (int(round(float(row[3]))), int(round(float(row[4])))), 1, (0, 255, 0), -1)
  cv2.imwrite("detection_image.jpg", detection_image)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-d", "--directory", default="./")
  args = parser.parse_args()
  if not os.path.isdir(args.directory):
    print("Directory " + args.directory + " does not exist.")
    sys.exit()
  view_detections(args.directory)
