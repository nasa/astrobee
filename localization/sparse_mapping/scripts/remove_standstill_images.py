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
Removes standstill images from a set of sequential images.
"""

import argparse
import subprocess

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "image_directory", help="Directory containing images. Images are assumed to be named in sequential order."
    )
    parser.add_argument("-m", "--max-standstill-mean-distance", type=float, default=0.01, help="Max mean distance for optical flow tracks between sequential images to be classified as a standstill pair.")
    args = parser.parse_args()
    remove_standstill_images_command = (
        "rosrun sparse_mapping remove_low_movement_images "
        + args.image_directory 
        + " -m "
        + str(args.max_standstill_mean_distance) 
    )
    subprocess.call(remove_standstill_images_command, shell=True)
