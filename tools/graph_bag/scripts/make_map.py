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
"""
Generates a map for a given input bagfile. Optionally 
merges new map from the provided input bagfile with an existing map.  
"""

import argparse
import os
import shutil
import sys

import utilities


def make_map(bagfile,
             map_name,
             world,
             robot_name,
             base_surf_map=None,
             maps_directory=None):
    merge_with_base_map = base_surf_map is not None and maps_directory is not None
    bag_images_dir = "bag_images_" + utilities.basename(bagfile)
    os.mkdir(bag_images_dir)
    bag_images = os.path.abspath(bag_images_dir)
    extract_images_command = (
        "rosrun localization_node extract_image_bag " + bagfile +
        " -use_timestamp_as_image_name -image_topic /mgt/img_sampler/nav_cam/image_record -output_directory "
        + bag_images)
    utilities.run_command_and_save_output(extract_images_command,
                                          "extract_images.txt")

    all_bag_images = os.path.join(bag_images, "*.jpg")
    select_images_command = (
        "rosrun sparse_mapping select_images -density_factor 1.4 " +
        all_bag_images)
    utilities.run_command_and_save_output(select_images_command,
                                          "select_images.txt")

    # Set environment variables
    home = os.path.expanduser("~")
    robot_config_file = os.path.join("config/robots", robot_name + ".config")
    astrobee_path = os.path.join(home, "astrobee/src/astrobee")
    os.environ["ASTROBEE_RESOURCE_DIR"] = os.path.join(astrobee_path,
                                                       "resources")
    os.environ["ASTROBEE_CONFIG_DIR"] = os.path.join(astrobee_path, "config")
    os.environ["ASTROBEE_ROBOT"] = os.path.join(astrobee_path,
                                                "config/robots/bumble.config")
    os.environ["ASTROBEE_WORLD"] = world

    # Build map
    bag_surf_map = map_name + ".map"
    build_map_command = (
        "rosrun sparse_mapping build_map " + all_bag_images + " -output_map " +
        bag_surf_map +
        " -feature_detection -feature_matching -track_building -incremental_ba -bundle_adjustment -histogram_equalization -num_subsequent_images 100"
    )
    utilities.run_command_and_save_output(build_map_command, "build_map.txt")

    if merge_with_base_map:
        merged_surf_map = map_name + ".surf.map"
        merge_map_command = (
            "rosrun sparse_mapping merge_maps " + base_surf_map + " " +
            bag_surf_map + " -output_map " + merged_surf_map +
            " -num_image_overlaps_at_endpoints 100000000 -skip_bundle_adjustment"
        )
        utilities.run_command_and_save_output(merge_map_command,
                                              "merge_map.txt")
        bag_surf_map = merged_surf_map

        # Link maps directory since conversion to BRISK map needs
        # image files to appear to be in correct relative path
        os.symlink(maps_directory, "maps")
        maps_bag_images = os.path.join("maps", bag_images_dir)
        os.symlink(bag_images, maps_bag_images)

    # Convert SURF to BRISK map
    # Get full path to output file to avoid permission errors when running
    # command in maps directory
    rebuild_output_file = os.path.join(os.getcwd(),
                                       "rebuild_map_as_brisk_map.txt")
    bag_brisk_map = map_name + ".brisk.map"
    shutil.copyfile(bag_surf_map, bag_brisk_map)
    bag_brisk_map_full_path = os.path.abspath(bag_brisk_map)
    bag_path = os.getcwd()
    if merge_with_base_map:
        os.chdir("maps")
    rebuild_map_command = (
        "rosrun sparse_mapping build_map -rebuild -histogram_equalization -output_map "
        + bag_brisk_map_full_path)
    utilities.run_command_and_save_output(rebuild_map_command,
                                          rebuild_output_file)
    # Use bag_path since relative commands would now be wrt maps directory simlink
    if merge_with_base_map:
        os.chdir(bag_path)

    # Create vocabdb
    bag_brisk_vocabdb_map = map_name + ".brisk.vocabdb.map"
    shutil.copyfile(bag_brisk_map, bag_brisk_vocabdb_map)
    add_vocabdb_command = (
        "rosrun sparse_mapping build_map -vocab_db -output_map " +
        bag_brisk_vocabdb_map)
    utilities.run_command_and_save_output(add_vocabdb_command,
                                          "build_vocabdb.txt")

    if merge_with_base_map:
        # Remove simlinks
        os.unlink(maps_bag_images)
        os.unlink("maps")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bagfile", help="Input bagfile to generate map for.")
    parser.add_argument(
        "-b",
        "--base-surf-map",
        default=None,
        help=
        "Optional existing map to merge new bag map into.  Should largely overlap area covered in input bagfile.",
    )
    parser.add_argument(
        "-m",
        "--maps-directory",
        default=None,
        help=
        "Location of images used for each bagfile use to generate optional base_surf_map. Only required if --base-surf-map provided.",
    )
    parser.add_argument("-o",
                        "--output-directory",
                        default="map_creation_output")
    parser.add_argument("-w", "--world", default="iss")
    parser.add_argument("-r", "--robot-name", default="bumble")
    parser.add_argument("-m", "--map-name", default="bag_map")

    args = parser.parse_args()
    if (args.base_surf_map or args.maps_directory
        ) and not (args.base_surf_map and args.maps_directory):
        print(
            "Only one of base surf map and maps directory provided. Provide both to merge with base surf map or neither to build standalone map."
        )
        sys.exit()
    if not os.path.isfile(args.bagfile):
        print("Bag file " + args.bagfile + " does not exist.")
        sys.exit()
    if arg.base_surf_map and not os.path.isfile(args.base_surf_map):
        print("Base surf map " + args.base_surf_map + " does not exist.")
        sys.exit()
    if args.maps_directory and not os.path.isdir(args.maps_directory):
        print("Maps directory " + args.maps_directory + " does not exist.")
        sys.exit()
    if os.path.isdir(args.output_directory):
        print("Output directory " + args.output_directory + " already exists.")
        sys.exit()

    bagfile = os.path.abspath(args.bagfile)
    base_surf_map = None
    maps_directory = None
    if args.base_surf_map:
        base_surf_map = os.path.abspath(args.base_surf_map)
        maps_directory = os.path.abspath(args.maps_directory)

    os.mkdir(args.output_directory)
    os.chdir(args.output_directory)

    make_map(bagfile, args.map_name, args.world, args.robot_name,
             base_surf_map, maps_directory)
