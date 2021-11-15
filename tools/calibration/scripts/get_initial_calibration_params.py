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
  print "\tProjection initialized to: %s" % camera.geometry.projection().getParameters().flatten()
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


def load_observation(csv_row, target, image):
  observation = acv.GridCalibrationTargetObservation(target)
  index = int(csv_row[0])
  image_point_x = float(csv_row[3])
  image_point_y = float(csv_row[4])
  print("index: " + str(index) + ",  x: "  + str(image_point_x) + ", y: " + str(image_point_y))
  observation.updateImagePoint(index, np.array([image_point_x, image_point_y]))
  #TODO(rsoussan): use real time?
  observation.setTime(acv.Time(1)) 
  observation.setImage(image)
  return observation

# TODO(rsoussan): Unify these with view_all_detections.py
def load_observations(directory, target):
  detection_files = [
    os.path.join(directory, filename)
    for filename in os.listdir(directory)
    if os.path.isfile(os.path.join(directory,filename)) and filename.endswith(".txt")
  ]
  if len(detection_files) == 0:
    print("No detection files found in directory.")
    sys.exit()

  observations = []
  for detection_file in detection_files:
    image_file = os.path.splitext(detection_file)[0] + ".jpg"
    print(image_file)
    image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    with open(detection_file) as detection_csvfile:
      reader = csv.reader(detection_csvfile, delimiter=" ")
      for row in reader:
        observation = load_observation(row, target, image)
        print("target rows: " + str(target.rows()))
        observations.append(observation) 
  return observations

def get_initial_calibration_params(directory, target_yaml):
  target_config = kc.CalibrationTargetParameters(target_yaml)
  # TODO: add this as param! load different models!
  camera_model = acvb.FovPinhole
  target_detector = kcc.TargetDetector(target_config, camera_model.geometry())
  observations = load_observations(directory, target_detector.detector.target())
  # TODO: need to fake dataset!!
  dataset = kc.BagImageDatasetReader('/home/rsoussan/iss_data/queen_calibration/2021_09_20/bags/20210920_1108_proc-204_step-2-nav_run-001.bag', '/hw/cam_nav')
  camera = kcc.CameraGeometry(camera_model, target_config, dataset)
  calibrate(observations, camera)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("target_yaml")
  parser.add_argument("-d", "--directory", default="./")
  args = parser.parse_args()
  if not os.path.isdir(args.directory):
    print("Directory " + args.directory + " does not exist.")
    sys.exit()
  get_initial_calibration_params(args.directory, args.target_yaml)
