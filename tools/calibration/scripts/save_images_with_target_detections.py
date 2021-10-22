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


class Corner:

  def __init__(self, corner_id, target_corner, image_corner):
    self.id = corner_id
    self.target_corner = np.array([target_corner[0], target_corner[1]])
    self.image_corner = np.array([image_corner[0], image_corner[1]])


class Corners:

  def __init__(self, observation):
    self.id_corner_map = {}
    target_corners = observation.getCornersTargetFrame()
    image_corners = observation.getCornersImageFrame()
    ids = observation.getCornersIdx()
    for i in range(len(target_corners)):
      corner = Corner(ids[i], target_corners[i], image_corners[i])
      self.id_corner_map[corner.id] = corner

  def similar(self, other_corners, threshold):
    # Make sure keys are the same
    if not set(self.id_corner_map.keys()) == set(other_corners.id_corner_map.keys()):
      return False

    norm_sums = 0
    for corner_id in self.id_corner_map.keys():
      a = (self.id_corner_map[corner_id].image_corner - other_corners.id_corner_map[corner_id].image_corner)
      image_diff_norm = np.linalg.norm(self.id_corner_map[corner_id].image_corner -
                                       other_corners.id_corner_map[corner_id].image_corner)
      norm_sums += image_diff_norm
    mean_norm = norm_sums / float(len(self.id_corner_map.keys()))
    if mean_norm < threshold:
      print("Ignoring image, mean " + str(mean_norm) + " below threshold " + str(threshold))
      return True
    return False


class AddedCorners:

  def __init__(self, threshold):
    self.corners = []
    self.threshold = threshold

  def add_corners(self, corners):
    self.corners.append(corners)

  def redundant(self, new_corners):
    if len(self.corners) == 0:
      return False
    for corners in self.corners:
      if corners.similar(new_corners, self.threshold):
        return True
    return False


def save_corners(observation, filename):
  target_corners = observation.getCornersTargetFrame()
  image_corners = observation.getCornersImageFrame()
  ids = observation.getCornersIdx()
  with open(filename, 'w') as corners_file:
    for i in range(len(target_corners)):
      corners_file.write('%0.17g %0.17g %0.17g %0.17g %0.17g\n' % (
        ids[i],
        target_corners[i][0],
        target_corners[i][1],
        image_corners[i][0],
        image_corners[i][1],
      ))


def save_images_from_dataset_with_target_detections(dataset, detector, output_directory, added_corners):
  for timestamp, image in dataset.readDataset():
    success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
    if success:
      # TODO(rsoussan): Why do no corners show up as success sometimes?
      if len(observation.getCornersIdx()) == 0:
        print("No Corners!")
        continue
      corners = Corners(observation)
      if added_corners.redundant(corners):
        continue
      else:
        added_corners.add_corners(corners)
      filepath = output_directory + '/' + os.path.splitext(os.path.basename(dataset.bagfile))[0] + '_' + str(
        timestamp.toSec())
      image_name = filepath + '.jpg'
      print("Saving " + filepath)
      cv2.imwrite(image_name, image)
      corners_name = filepath + '.txt'
      save_corners(observation, corners_name)


def save_images_from_bags_directory_with_target_detections(bags_directory, target_yaml, cam_topic, output_directory,
                                                           threshold):
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
  added_corners = AddedCorners(threshold)
  for bag_name in bag_names:
    dataset = kc.BagImageDatasetReader(bag_name, cam_topic)
    camera_geometry = kcc.CameraGeometry(camera_model, target_config, dataset)
    save_images_from_dataset_with_target_detections(dataset, target_detector.detector, args.output_directory,
                                                    added_corners)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-d", "--directory", default="./")
  parser.add_argument("-o", "--output-directory", default="./images_with_target_detections")
  parser.add_argument("--cam-topic", default="/hw/cam_nav")
  parser.add_argument("-t", "--target-yaml")
  parser.add_argument("--threshold", type=float, default=100)
  args = parser.parse_args()
  save_images_from_bags_directory_with_target_detections(args.directory, args.target_yaml, args.cam_topic,
                                                         args.output_directory, args.threshold)
