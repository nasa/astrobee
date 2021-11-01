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
import shutil

import os
import subprocess
import sys

#TODO(rsoussan): Put this somewhere common -> utils?
def run_command_and_save_output(command, output_filename):
  with open(output_filename, 'w') as output_file:
    subprocess.call(command, shell=True, stdout=output_file, stderr=output_file)

def create_groundtruth(bagfile, base_surf_map, world, robot_name):
  os.mkdir("images")
  extract_images_command = "rosrun localization_node extract_image_bag " + bagfile + " -use_timestamp_as_image_name -image_topic /mgt/img_sampler/nav_cam/image_record -output_directory images"
  run_command_and_save_output(extract_images_command, 'extract_images.txt')

  select_images_command = "rosrun sparse_mapping select_images -density_factor 1.4 images/*.jpg"
  run_command_and_save_output(select_images_command, 'select_images.txt')


  return
  # Set environment variables
  home = os.path.expanduser('~')
  robot_config_file = os.path.join('config/robots', robot_name + '.config')
  astrobee_path = os.path.join(home, 'astrobee/astrobee')
  os.environ['ASTROBEE_RESOURCE_DIR'] = os.path.join(astrobee_path, 'resources')
  os.environ['ASTROBEE_CONFIG_DIR'] = os.path.join(astrobee_path, 'config')
  os.environ['ASTROBEE_ROBOT'] = os.path.join(astrobee_path, 'config/robots/bumble.config')
  os.environ['ASTROBEE_WORLD'] = world

  # Build groundtruth
  build_map_command = 'rosrun sparse_mapping build_map images/*jpg -output_map groundtruth.map -feature_detection -feature_matching -track_building -incremental_ba -bundle_adjustment -histogram_equalization -num_subsequent_images 100'
  run_command_and_save_output(build_map_command, 'build_map.txt')

  # Merge with base map
  merge_map_command = 'rosrun sparse_mapping merge_maps ' + base_surf_map + ' groundtruth.map -output_map groundtruth.surf.map -num_image_overlaps_at_endpoints 100000000 -skip_bundle_adjustment'
  run_command_and_save_output(merge_map_command, 'merge_map.txt')

  ## Convert SURF to BRISK map
  shutil.copyfile("groundtruth.surf.map", "groundtruth.brisk.map")
  rebuild_map_command = 'rosrun sparse_mapping build_map -rebuild -histogram_equalization -output_map groundtruth.brisk.map'
  run_command_and_save_output(rebuild_map_command, 'rebuild_map_as_brisk_map.txt')

  # TODO: build vocab!

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile')
  parser.add_argument('base_surf_map')
  parser.add_argument('-o', '--output-directory', default='groundtruth_creation_output')
  parser.add_argument('-w', '--world', default='iss')
  parser.add_argument('-r', '--robot-name', default='bumble')

  args = parser.parse_args()
  if not os.path.isfile(args.bagfile):
    print('Bag file ' + args.bagfile + ' does not exist.')
    sys.exit()
  if not os.path.isfile(args.base_surf_map):
    print('Base surf map ' + args.base_surf_map + ' does not exist.')
    sys.exit()
  if os.path.isdir(args.output_directory):
    print('Output directory ' + args.output_directory + ' already exists.')
    sys.exit()

  bagfile = os.path.abspath(args.bagfile)
  base_surf_map = os.path.abspath(args.base_surf_map)

  os.mkdir(args.output_directory)
  os.chdir(args.output_directory)

  create_groundtruth(bagfile, base_surf_map, args.world, args.robot_name)
