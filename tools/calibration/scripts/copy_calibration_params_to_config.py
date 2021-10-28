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
import fileinput
import os
import sys

import cv2
import numpy as np


def make_calibration_params_string(calibration_filename):
  fx = 0
  fy = 0
  px = 0
  py = 0
  distortion = []
  with open(calibration_filename) as calibration_file:
    reader = csv.reader(calibration_file, delimiter=" ")
    rows = list(reader)
    # Row order is focal lengths, principal points, distortion params
    fx = rows[0][0]
    fy = rows[0][1]
    px = rows[1][0]
    py = rows[1][1]
    for val in rows[2]:
      distortion.append(val)
  params_string = '    distortion_coeff = '
  if len(distortion) > 1:
    params_string += '{'
    for i in range(len(distortion)):
      params_string += distortion[i]
      if i < (len(distortion) - 1):
        params_string += ', '
    params_string += '},'
  else:
    params_string += distortion[0] + ','
  params_string += '\n'
  params_string += '    intrinsic_matrix = {\n'
  params_string += '      ' + str(fx) + ', ' + '0.0' + ', ' + str(px) + ',\n'
  params_string += '      ' + '0.0' + ', ' + str(fy) + ', ' + str(py) + ',\n'
  params_string += '      ' + '0.0, 0.0, 1.0\n'
  params_string += '    },'
  return params_string


def copy_calibration_params_to_config(config, camera_name, calibration_file):
  camera_name_line = camera_name + ' = {'
  new_calibration_string = make_calibration_params_string(calibration_file)
  config_file = fileinput.input(config, inplace=1)
  for line in config_file:
    print line,
    if line.strip() == camera_name_line:
      print(new_calibration_string)
      [config_file.next() for x in range(6)]


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--config")
  parser.add_argument("--camera-name")
  parser.add_argument("-c", "--calibration_file", default="calibrated_params.txt")
  args = parser.parse_args()
  if not os.path.isfile(args.config):
    print("Config file " + args.config + " does not exist.")
    sys.exit()
  if not os.path.isfile(args.calibration_file):
    print("Calibration file " + args.calibration_file + " does not exist.")
    sys.exit()
  copy_calibration_params_to_config(args.config, args.camera_name, args.calibration_file)
