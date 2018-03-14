#!/usr/bin/python
#
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
import os.path
import re
import subprocess
import sys
import yaml

# returns intrinsics and distortion from yaml file
def read_yaml(filename, cameras):
  intrinsic = None
  dist = None
  with open(filename, 'r') as f:
    dist = []
    intrinsic = []
    try:
      d = yaml.load(f)
      for cam in cameras:
        dist.append(d[cam]['distortion_coeffs'])
        intrinsic.append(d[cam]['intrinsics'])
    except yaml.YAMLError as exc:
      print >> sys.stderr, exc
  return (intrinsic, dist)

def find_close_bracket(s):
  assert(s[0] == '{')
  count = 1
  for i in range(1, len(s)):
    if s[i] == '{':
      count += 1
    elif s[i] == '}':
      count -= 1
    if count == 0:
      return i
  return -1

def lua_replace_distortion(filename, camera, intrinsics, distortion):
  try:
    with open(filename, 'r') as f:
      contents = f.read()
  except IOError:
    print >> sys.stderr, 'Failed to open lua file.'
    return True
  prog = re.compile(re.escape(camera) + '\s*=\s*\{')
  match = re.search(prog, contents)
  if not match:
    print >> sys.stderr, 'Camera %s not found in lua file %s.' % (camera, filename)
    return True
  open_bracket = match.end() - 1
  close_bracket = find_close_bracket(contents[open_bracket:])
  if close_bracket < 0:
    print >> sys.stderr, 'Camera config did not terminate.'
    return True
  close_bracket = open_bracket + close_bracket
  camera_text = contents[open_bracket:close_bracket+1]

  prog = re.compile('distortion_coeff\s*=\s*[+-]?([0-9]*[.])?[0-9]+')
  
  if type(distortion) is float:
    (camera_text, replacements) = re.subn(prog, 'distortion_coeff = %g' % (distortion), camera_text)
  else:
    (camera_text, replacements) = re.subn(prog, 'distortion_coeff = {%g, %g, %g, %g}' % (distortion), camera_text) 

  if replacements != 1:
    print >> sys.stderr, 'Failed to replace distortion.'
    return True

  prog = re.compile('intrinsic_matrix\s*=\s*\{.*?\}', re.DOTALL)
  
  intrinsic_string = 'intrinsic_matrix = {\n      %.8g, 0.0, %.8g,\n      0.0, %.8g, %.8g,\n      0.0, 0.0, 1.0\n    }' % (\
      intrinsics[0], intrinsics[2], intrinsics[1], intrinsics[3])
  (camera_text, replacements) = re.subn(prog, intrinsic_string, camera_text)
  if replacements != 1:
    print >> sys.stderr, 'Failed to replace intrinsics matrix.'
    return True

  output_text = contents[:open_bracket] + camera_text + contents[close_bracket+1:]
  try:
    with open(filename, 'w') as f:
      f.write(output_text)
  except IOError:
    print >> sys.stderr, 'Failed to open lua file.'
    return True
  return False

def calibrate_camera(bag, target_file, dock_cam=False, depth_cam=False, verbose=False, model='pinhole-fov',model_depth='pinhole-radtan'):
  CAMERA_TOPIC = '/hw/cam_dock' if dock_cam else '/hw/cam_nav'
  
  if CAMERA_TOPIC == '/hw/cam_dock':
    CAMERA_TOPIC_DEPTH = '/hw/depth_perch/extended/amplitude' if depth_cam else ''
  else:
    CAMERA_TOPIC_DEPTH = '/hw/depth_haz/extended/amplitude' if depth_cam else ''

  extra_flags = ' --verbose' if verbose else ' --dont-show-report'
  
  if not os.path.isfile(bag):
    print >> sys.stderr, 'Bag file %s does not exist.' % (bag)
    return None
  if not os.path.isfile(target_file):
    print >> sys.stderr, 'Target file %s does not exist.' % (target_file)
    return None

  bag_dir = os.path.dirname(os.path.abspath(bag))
  bag_file = os.path.basename(bag)
  bag_name = os.path.splitext(bag_file)[0]
 
  try:
    output_arg = None if verbose else open(os.devnull, 'w')
  except IOError:
    print >> sys.stderr, 'Failed to open devnull.'
    return None
  if depth_cam:
    ret = subprocess.call(('rosrun kalibr kalibr_calibrate_cameras --topics %s %s --models %s %s --target %s --bag %s%s' %
           (CAMERA_TOPIC, CAMERA_TOPIC_DEPTH, model, model_depth, target_file, bag_file, extra_flags)).split(),
           cwd=bag_dir, stdout=output_arg, stderr=output_arg)
  else:
    ret = subprocess.call(('rosrun kalibr kalibr_calibrate_cameras --topics %s --models %s --target %s --bag %s%s' %
           (CAMERA_TOPIC, model, target_file, bag_file, extra_flags)).split(),
           cwd=bag_dir, stdout=output_arg, stderr=output_arg)

  if ret != 0:
    print >> sys.stderr, 'Failed to calibrate camera from bag file.'
    return None

  return bag_dir + '/camchain-' + bag_name + '.yaml' #Bag dir or data dir

def main():
  parser = argparse.ArgumentParser(description='Calibrate intrinsic parameters.')
  parser.add_argument('-d', '--dock_cam', dest='dock_cam', action='store_true', help='Calibrate dock camera.')
  parser.add_argument('-depth', '--depth_cam', dest='depth_cam', action='store_true', help='Calibrate respective depth camera.')
  parser.add_argument('-v', '--verbose',  dest='verbose', action='store_true', help='Verbose mode.')
  parser.add_argument('robot', help='The name of the robot to configure (i.e., put p4d to edit p4d.config).')
  parser.add_argument('bag', help='The bag file with calibration data.')
  args = parser.parse_args()

  #Setup Files
  SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
  target_file = SCRIPT_DIR + '/data/granite_april_tag.yaml'
  yaml_file = calibrate_camera(args.bag, target_file, dock_cam=args.dock_cam, depth_cam=args.depth_cam, verbose=args.verbose)

  if yaml_file is None:
    print >> sys.stderr, 'Failed to run calibration.'
    return 

  if args.depth_cam:
    (intrinsics, distortion) = read_yaml(yaml_file, ['cam0', 'cam1'])
    if len(intrinsics) < 2 or len(distortion) < 2:
      print >> sys.stderr, 'Failed to read depth camera parameters.'
      return
  else:
    (intrinsics, distortion) = read_yaml(yaml_file,  ['cam0'])
  
  if not intrinsics or not distortion:
    print >> sys.stderr, 'Failed to read camera intrinsics.'
    return

  config_file = SCRIPT_DIR + '/../../astrobee/config/robots/' + args.robot + '.config'
  if lua_replace_distortion(config_file, 'dock_cam' if args.dock_cam else 'nav_cam', intrinsics[0], distortion[0][0]):
    print >> sys.stderr, 'Failed to update config file with HD camera intrinsics.'
    return
  
  #Replace Intrinsics of Depth Cameras
  
  #if args.depth_cam:
  #  if lua_replace_distortion(config_file, 'perch_cam' if args.dock_cam else 'haz_cam', intrinsics[1], distortion[1]):
  #    print >> sys.stderr, 'Failed to update config file with depth camera intrinsics.'
  #    return

if __name__ == '__main__':
  main()
