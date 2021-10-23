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

import aslam_cv_backend as acvb
import aslam_cv as acv
import kalibr_common as kc
import kalibr_camera_calibration as kcc


def calibrate(observations, camera):
  #obtain focal length guess
  success = camera.geometry.initializeIntrinsics(observations)
  if not success:
    print("initialization of focal length failed.")
    sys.exit()
  
  #optimize for intrinsics & distortion    
  success = kcc.calibrateIntrinsics(camera, observations)
  if not success:
      print("initialization of intrinsics failed.")
      sys.exit()
  print "\tProjection initialized to: %s" % camera.geometry.projection().getParameters().flatten()
  print "\tDistortion initialized to: %s" % camera.geometry.projection().distortion().getParameters().flatten()


def load_observation(csv_row, target):
  observation = acv.GridCalibrationTargetObservation(target)
  index = int(csv_row[0])
  image_point_x = float(csv_row[3])
  image_point_y = float(csv_row[4])
  observation.updateImagePoint(index, np.array([image_point_x, image_point_y]))
  #TODO(rsoussan): use real time?
  observation.setTime(acv.Time(1))
  return observation

# TODO(rsoussan): Unify these with view_all_detections.py
def load_observations(directory, target):
  detection_files = [
    directory + filename
    for filename in os.listdir(directory)
    if os.path.isfile(directory + filename) and filename.endswith(".txt")
  ]
  if len(detection_files) == 0:
    print("No detection files found in directory.")
    sys.exit()

  observations = []
  for detection_file in detection_files:
    with open(detection_file) as detection_csvfile:
      reader = csv.reader(detection_csvfile, delimiter=" ")
      for row in reader:
        observation = load_observation(row, target)
        observations.append(observation) 
  return observations

def calibrate_with_corners(directory, target_yaml):
  target_config = kc.CalibrationTargetParameters(target_yaml)
  # TODO: add this as param! load different models!
  camera_model = acvb.DistortedPinhole
  target_detector = kcc.TargetDetector(target_config, camera_model.geometry())
  observations = load_observations(directory, target_detector.detector.target())
  # TODO: need to fake dataset!!
  dataset = kc.BagImageDatasetReader('/home/rsoussan/iss_data/queen_test/large_bag_pruned.bag', '/cam0/image_raw')
  camera = kcc.CameraGeometry(camera_model, target_config, dataset)
  calibrate(observations, camera)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-d", "--directory", default="./")
  parser.add_argument("-t", "--target-yaml")
  args = parser.parse_args()
  if not os.path.isdir(args.directory):
    print("Directory " + args.directory + " does not exist.")
    sys.exit()
  calibrate_with_corners(args.directory, args.target_yaml)
