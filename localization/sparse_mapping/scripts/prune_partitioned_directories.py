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
Removes valid or rotation subdirectories with no files and moves too small subdirectories to invalid
"""

import argparse
import os
import shutil


def remove_or_move_directories(directory, invalid_path, min_directory_size):
    subdirectories = []
    try:
        _, subdirectories, _ = next(os.walk(directory))
    except:
        return 
    for subdirectory in subdirectories:
        if subdirectory == "invalid" or subdirectory == "rotation":
            continue
        subdirectory_path = os.path.join(directory, subdirectory)
        num_files = len([name for name in os.listdir(subdirectory_path) if os.path.isfile(os.path.join(subdirectory_path, name))])
        if num_files == 0:
            print("Removing empty directory: " + subdirectory_path)
            os.rmdir(subdirectory_path)
        elif num_files < min_directory_size:
            invalid_path = os.path.join(invalid_path, subdirectory)
            print("Moving small directory: " + subdirectory_path + ", num images: " + str(num_files))
            shutil.move(subdirectory_path, invalid_path)
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "image_directory", help="Directory containing images. Images are assumed to be named in sequential order."
    )
    parser.add_argument(
        "-m", "--min-directory-size", type=float, default=10, help="Min valid or rotation directory size, otherwise the directory is moved to invalid."
    )
    args = parser.parse_args()
    invalid_path = os.path.join(args.image_directory, "invalid")
    remove_or_move_directories(args.image_directory, invalid_path, args.min_directory_size)
    remove_or_move_directories(os.path.join(args.image_directory, "rotation"), invalid_path, args.min_directory_size)
