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
Removes standstill images, partitions images into valid, rotation, and invalid sequences, and finally removes low-movement images from each of these sequences. This helps limit the number of images before running bundle-adjustment while also separating the images into sequences to allow for a more intelligent bundle-adjustment strategy.
For more information on each script called, see:
'rosrun sparse_mapping remove_standstill_images -h'
'rosrun sparse_mapping partition_image_sequences -h'
'rosrun sparse_mapping remove_low_movement_images -h'
"""

import argparse
import subprocess

import localization_common.utilities as lu

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "image_directory", help="Directory containing images. Images are assumed to be named in sequential order."
    )
    parser.add_argument(
        "config-path", help="Full path to astrobee/src/astrobee directory location, e.g. ~/astrobee/src/astrobee.")
    args = parser.parse_args()

    remove_standstill_images_command = (
        "rosrun sparse_mapping remove_standstill_images "
        + args.image_directory 
    )
    lu.run_command_and_save_output(remove_standstill_images_command, "remove_standstill_images.txt")

    partition_image_sequences_command = (
        "rosrun sparse_mapping partition_image_sequences "
        + args.image_directory 
        + " "
        + args.config_path
    )
    lu.run_command_and_save_output(partition_image_sequences_command, "partition_image_sequences.txt")

    remove_low_movement_images_command = (
        "rosrun sparse_mapping remove_low_movement_images "
        + args.image_directory 
    )
    lu.run_command_and_save_output(remove_low_movement_images_command, "remove_low_movement_images.txt")
