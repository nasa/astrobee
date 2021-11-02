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
import shutil
import sys

import utilities


def create_groundtruth(
    bagfile, base_surf_map, maps_directory, map_name, world, robot_name
):
    os.mkdir("gt_images")
    gt_images = os.path.abspath("gt_images")
    extract_images_command = (
        "rosrun localization_node extract_image_bag "
        + bagfile
        + " -use_timestamp_as_image_name -image_topic /mgt/img_sampler/nav_cam/image_record -output_directory gt_images"
    )
    utilities.run_command_and_save_output(extract_images_command, "extract_images.txt")

    select_images_command = (
        "rosrun sparse_mapping select_images -density_factor 1.4 gt_images/*.jpg"
    )
    utilities.run_command_and_save_output(select_images_command, "select_images.txt")

    # Set environment variables
    home = os.path.expanduser("~")
    robot_config_file = os.path.join("config/robots", robot_name + ".config")
    astrobee_path = os.path.join(home, "astrobee/astrobee")
    os.environ["ASTROBEE_RESOURCE_DIR"] = os.path.join(astrobee_path, "resources")
    os.environ["ASTROBEE_CONFIG_DIR"] = os.path.join(astrobee_path, "config")
    os.environ["ASTROBEE_ROBOT"] = os.path.join(
        astrobee_path, "config/robots/bumble.config"
    )
    os.environ["ASTROBEE_WORLD"] = world

    # Build groundtruth
    groundtruth_map = map_name + ".map"
    build_map_command = (
        "rosrun sparse_mapping build_map gt_images/*jpg -output_map "
        + groundtruth_map
        + " -feature_detection -feature_matching -track_building -incremental_ba -bundle_adjustment -histogram_equalization -num_subsequent_images 100"
    )
    utilities.run_command_and_save_output(build_map_command, "build_map.txt")

    # Merge with base map
    groundtruth_surf_map = map_name + ".surf.map"
    merge_map_command = (
        "rosrun sparse_mapping merge_maps "
        + base_surf_map
        + " "
        + groundtruth_map
        + " -output_map "
        + groundtruth_surf_map
        + " -num_image_overlaps_at_endpoints 100000000 -skip_bundle_adjustment"
    )
    utilities.run_command_and_save_output(merge_map_command, "merge_map.txt")

    # Link maps directory since conversion to BRISK map needs
    # image files to appear to be in correct relative path
    os.symlink(maps_directory, "maps")
    os.symlink(gt_images, "maps/gt_images")

    # Convert SURF to BRISK map
    # Get full path to output file to avoid permission errors when running
    # command in maps directory
    rebuild_output_file = os.path.join(os.getcwd(), "rebuild_map_as_brisk_map.txt")
    groundtruth_brisk_map = map_name + ".brisk.map"
    shutil.copyfile(groundtruth_surf_map, groundtruth_brisk_map)
    groundtruth_brisk_map_full_path = os.path.abspath(groundtruth_brisk_map)
    gt_path = os.getcwd()
    os.chdir("maps")
    rebuild_map_command = (
        "rosrun sparse_mapping build_map -rebuild -histogram_equalization -output_map "
        + groundtruth_brisk_map_full_path
    )
    utilities.run_command_and_save_output(rebuild_map_command, rebuild_output_file)
    # Use gt_path since relative commands would now be wrt maps directory simlink
    os.chdir(gt_path)

    # Create vocabdb
    groundtruth_brisk_vocabdb_map = map_name + ".brisk.vocabdb.map"
    shutil.copyfile(groundtruth_brisk_map, groundtruth_brisk_vocabdb_map)
    add_vocabdb_command = (
        "rosrun sparse_mapping build_map -vocab_db -output_map "
        + groundtruth_brisk_vocabdb_map
    )
    utilities.run_command_and_save_output(add_vocabdb_command, "build_vocabdb.txt")

    # Remove simlinks
    os.unlink("maps/gt_images")
    os.unlink("maps")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile")
    parser.add_argument("base_surf_map")
    parser.add_argument("maps_directory")
    parser.add_argument(
        "-o", "--output-directory", default="groundtruth_creation_output"
    )
    parser.add_argument("-w", "--world", default="iss")
    parser.add_argument("-r", "--robot-name", default="bumble")
    parser.add_argument("-m", "--map-name", default="groundtruth")

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print("Bag file " + args.bagfile + " does not exist.")
        sys.exit()
    if not os.path.isfile(args.base_surf_map):
        print("Base surf map " + args.base_surf_map + " does not exist.")
        sys.exit()
    if not os.path.isdir(args.maps_directory):
        print("Maps directory " + args.maps_directory + " does not exist.")
        sys.exit()
    if os.path.isdir(args.output_directory):
        print("Output directory " + args.output_directory + " already exists.")
        sys.exit()

    bagfile = os.path.abspath(args.bagfile)
    base_surf_map = os.path.abspath(args.base_surf_map)
    maps_directory = os.path.abspath(args.maps_directory)

    os.mkdir(args.output_directory)
    os.chdir(args.output_directory)

    create_groundtruth(
        bagfile,
        base_surf_map,
        maps_directory,
        args.map_name,
        args.world,
        args.robot_name,
    )
