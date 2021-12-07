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
"""
Calibrates camera intrinsics using provided config file and target detections
and updates the provided config file with the calibration results.
Optionally undistorts images for provided directory with calibration results.
"""

import argparse
import os
import subprocess
import sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "corners_directory", help="Path to target detections to use for calibration."
    )
    parser.add_argument("config_path")
    parser.add_argument("robot_config_file")
    parser.add_argument("-w", "--world", default="iss")
    parser.add_argument(
        "-o", "--output-directory", default="intrinsics_calibration_output"
    )
    parser.add_argument("-u", "--undistort-images", action="store_true")
    parser.add_argument("-p", "--plot-histogram-errors", action="store_true")
    parser.add_argument(
        "-i",
        "--image-directory-to-undistort",
        default=None,
        help="Optional seperate directory of images to undistort with calibration results.  Ensure that the --undistort-images flag is passed.",
    )
    # TODO(rsoussan): Get these from calibration config file
    parser.add_argument(
        "-c", "--camera-name", default="nav_cam", help="Used for optional undistortion."
    )
    parser.add_argument(
        "-d", "--distortion-type", default="fov", help="Used for optional undistortion."
    )
    args = parser.parse_args()
    if not os.path.isdir(args.corners_directory):
        print("Corners directory " + args.corners_directory + " does not exist.")
        sys.exit()
    if os.path.isdir(args.output_directory):
        print("Output directory " + args.output_directory + " already exists.")
        sys.exit()
    if args.image_directory_to_undistort and not os.path.isdir(
        args.image_directory_to_undistort
    ):
        print(
            "Image directory to undistort "
            + args.image_directory_to_undistort
            + " does not exist."
        )
        sys.exit()

    # Get absolute paths to directories so we can change to the output directory without breaking other directory paths
    corners_directory = os.path.abspath(args.corners_directory)
    images_directory_to_undistort = (
        os.path.abspath(args.image_directory_to_undistort)
        if args.image_directory_to_undistort
        else None
    )

    os.mkdir(args.output_directory)
    os.chdir(args.output_directory)

    calibration_command = (
        "rosrun calibration run_camera_target_based_intrinsics_calibrator "
        + corners_directory
        + " -c "
        + args.config_path
        + " -r "
        + args.robot_config_file
        + " -w "
        + args.world
    )
    with open("calibration_output.txt", "w") as output_file:
        subprocess.call(
            calibration_command, shell=True, stdout=output_file, stderr=output_file
        )

    if args.plot_histogram_errors:
        histogram_plotter_command = "rosrun calibration make_error_histograms.py"
        os.system(histogram_plotter_command)

    if not args.undistort_images:
        sys.exit()

    robot_config_file_full_path = os.path.join(args.config_path, args.robot_config_file)
    copy_calibration_params_command = (
        "rosrun calibration copy_calibration_params_to_config.py --config "
        + robot_config_file_full_path
        + " --camera-name "
        + args.camera_name
    )
    os.system(copy_calibration_params_command)

    undistort_directory = (
        images_directory_to_undistort
        if images_directory_to_undistort
        else corners_directory
    )
    undistort_images_command = (
        "rosrun calibration create_undistorted_images "
        + undistort_directory
        + " "
        + args.distortion_type
        + " "
        + args.camera_name
        + " "
        + args.config_path
        + " --robot-config-file "
        + args.robot_config_file
        + " -w "
        + args.world
    )
    os.system(undistort_images_command)
