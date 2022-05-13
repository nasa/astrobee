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
Generates a new brisk map with a vocab database using the provided surf map.
"""

import argparse
import os
import shutil
import sys

import localization_common.utilities as lu


def make_brisk_map(
    surf_map,
    world,
    robot_name,
    map_directory = None
):
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


    # Convert SURF to BRISK map
    # Get full path to output file to avoid permission errors when running
    # command in maps directory
    map_name = lu.basename(surf_map)
    rebuild_output_file = os.path.join(os.getcwd(), "rebuild_map_as_brisk_map.txt")
    brisk_map = map_name + ".brisk.map"
    shutil.copyfile(surf_map, brisk_map)
    brisk_map_full_path = os.path.abspath(brisk_map)
    path = os.getcwd()
    # Change to map directory so relative image locations are correct if necessary
    if map_directory:
        os.chdir(map_directory)
    build_brisk_map_command = (
        "rosrun sparse_mapping build_map -rebuild -histogram_equalization -output_map "
        + brisk_map_full_path
    )
    lu.run_command_and_save_output(build_brisk_map_command, build_brisk_output_file)
    # Change back to original directory so final map is saved there
    if map_directory:
        os.chdir(path)

    # Create vocabdb
    brisk_vocabdb_map = map_name + ".brisk.vocabdb.map"
    shutil.copyfile(brisk_map, brisk_vocabdb_map)
    add_vocabdb_command = (
        "rosrun sparse_mapping build_map -vocab_db -output_map " + brisk_vocabdb_map
    )
    lu.run_command_and_save_output(add_vocabdb_command, "build_vocabdb.txt")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("surf_map", help="Input SURF map to generate the BRISK map for.")
    parser.add_argument("-w", "--world", default="iss")
    parser.add_argument("-r", "--robot-name", default="bumble")
    parser.add_argument("-d", "--map-directory", default=None, help="Location where surf map was created, needed to load images for BRISK map building since relative paths are used during map creation for the image locations. Defaults to current working directory.")

    args = parser.parse_args()
    if not os.path.isfile(args.surf_map):
        print("SURF map " + args.surf_map + " does not exist.")
        sys.exit()

    if args.map_directory and not os.path.isdir(args.map_directory):
        print("Map directory " + args.map_directory + " does not exist.")
        sys.exit()

    surf_map = os.path.abspath(args.surf_map)
    if args.map_directory:
        args.map_directory = os.path.abspath(args.map_directory)
    make_brisk_map(
        surf_map,
        args.world,
        args.robot_name,
        args.map_directory
    )
