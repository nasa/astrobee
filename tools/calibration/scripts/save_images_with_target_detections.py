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

import get_bags_with_topic

import cv2
import numpy as np

import aslam_cv_backend as acvb
import kalibr_common as kc
import kalibr_camera_calibration as kcc


def save_images_from_dataset_with_target_detections(dataset, detector, output_directory):
  for timestamp, image in dataset.readDataset():
    success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
    if success:
      cv2.imshow('image', image)
      cv2.waitKey(0)
      #save_image(image, observation, output_directory) #TODO: make this function!!


def save_images_from_bags_directory_with_target_detections(bags_directory, target_yaml, cam_topic, output_directory):
  bag_names = get_bags_with_topic.find_bags_with_topic(bags_directory, cam_topic)
  if len(bag_names) == 0:
    print("No bag files with topic " + cam_topic + " found.")
    sys.exit()
  else:
    print(("Found " + str(len(bag_names)) + " bag files with " + cam_topic + " topic."))
    for bag_name in bag_names:
      print(bag_name)

  if not os.path.isdir(output_directory):
    os.mkdir(output_directory)

  # Camera model isn't used for target detection so any model should suffice
  camera_model = acvb.DistortedPinhole
  target_config = kc.CalibrationTargetParameters(target_yaml)
  target_detector = kcc.TargetDetector(target_config, camera_model.geometry())
  for bag_name in bag_names:
    dataset = kc.BagImageDatasetReader(bag_name, cam_topic)
    camera_geometry = kcc.CameraGeometry(camera_model, target_config, dataset)
    save_images_from_dataset_with_target_detections(dataset, target_detector.detector, args.output_directory)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-d", "--directory", default="./")
  parser.add_argument("-o", "--output-directory", default="./images_with_target_detections")
  parser.add_argument("--cam-topic", default="/hw/cam_nav")
  parser.add_argument("-t", "--target-yaml")
  args = parser.parse_args()
  save_images_from_bags_directory_with_target_detections(args.directory, args.target_yaml, args.cam_topic,
                                                         args.output_directory)
