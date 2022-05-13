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
Extracts images, removes low movements images, and generates a map for a given input bagfile. 
"""

import argparse
import os
import shutil
import sys

import localization_common.utilities as lu


def make_surf_map(
    bagfile,
    world,
    robot_name,
):
    basename = lu.basename(bagfile)
    bag_images_dir = basename + "_bag_images"
    os.mkdir(bag_images_dir)
    bag_images_dir_path = os.path.abspath(bag_images_dir)
    extract_images_command = (
        "rosrun localization_node extract_image_bag "
        + bagfile
        + " -use_timestamp_as_image_name -image_topic /mgt/img_sampler/nav_cam/image_record -output_directory "
        + bag_images_dir_path
    )
    lu.run_command_and_save_output(
        extract_images_command, basename + "_extract_images.txt"
    )

    remove_low_movement_images_command = (
        "rosrun sparse_mapping remove_low_movement_images " + bag_images_dir_path
    )
    lu.run_command_and_save_output(
        remove_low_movement_images_command, basename + "_remove_low_movement_images.txt"
    )

    # Set environment variables
    home = os.path.expanduser("~")
    robot_config_file = os.path.join("config/robots", robot_name + ".config")
    astrobee_path = os.path.join(home, "astrobee/src/astrobee")
    os.environ["ASTROBEE_RESOURCE_DIR"] = os.path.join(astrobee_path, "resources")
    os.environ["ASTROBEE_CONFIG_DIR"] = os.path.join(astrobee_path, "config")
    os.environ["ASTROBEE_ROBOT"] = os.path.join(
        astrobee_path, robot_config_file
    )
    os.environ["ASTROBEE_WORLD"] = world

    # Build map
    relative_bag_images_path = os.path.relpath(bag_images_dir)
    bag_images = os.path.join(relative_bag_images_path, "*.jpg")
    map_name = basename + ".map"
    build_map_command = (
        "rosrun sparse_mapping build_map "
        + bag_images
        + " -output_map "
        + map_name
        + " -feature_detection -feature_matching -track_building -incremental_ba -bundle_adjustment -histogram_equalization -num_subsequent_images 100000000"
    )
    lu.run_command_and_save_output(build_map_command, basename + "_build_map.txt")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile to generate map for.")
    parser.add_argument("-w", "--world", default="iss")
    parser.add_argument("-r", "--robot-name", default="bumble")

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print("Bag file " + args.bagfile + " does not exist.")
        sys.exit()

    bagfile = os.path.abspath(args.bagfile)
    make_surf_map(
        bagfile,
        args.world,
        args.robot_name,
    )
