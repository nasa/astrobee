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

import numpy as np
import numpy.linalg
import yaml
from calibration_utils import *
from tf import transformations


# returns intrinsics, distortion, and transforms between cameras from a yaml file
def read_yaml(filename, cameras):
    dist = []
    intrinsics = []
    transforms = []
    with open(filename, "r") as f:
        try:
            d = yaml.load(f)
            for cam in cameras:
                dist.append(d[cam]["distortion_coeffs"])
                intrinsics.append(d[cam]["intrinsics"])
                if cam == "cam0":
                    # No transform from cam0 to itself, just use a placeholder
                    transforms.append(np.array([[]]))
                else:
                    # Transform from current camera to previous one
                    transforms.append(np.array(d[cam]["T_cn_cnm1"]))
        except yaml.YAMLError as exc:
            print(exc, file=sys.stderr)

    return (intrinsics, dist, transforms)


def update_intrinsics(filename, camera, intrinsics, distortion):
    try:
        with open(filename, "r") as f:
            contents = f.read()
    except IOError:
        print("Failed to open lua file.", file=sys.stderr)
        return True
    prog = re.compile(re.escape(camera) + "\s*=\s*\{")
    match = re.search(prog, contents)
    if not match:
        print(
            "Camera %s not found in lua file %s." % (camera, filename), file=sys.stderr
        )
        return True
    open_bracket = match.end() - 1
    close_bracket = find_close_bracket(contents[open_bracket:])
    if close_bracket < 0:
        print("Camera config did not terminate.", file=sys.stderr)
        return True
    close_bracket = open_bracket + close_bracket
    camera_text = contents[open_bracket : close_bracket + 1]

    # Update the distortion
    prog = re.compile("distortion_coeff\s*=\s*.*?\n")
    if type(distortion) is float:
        (camera_text, replacements) = re.subn(
            prog, "distortion_coeff = %g,\n" % (distortion), camera_text
        )
    else:
        (camera_text, replacements) = re.subn(
            prog,
            "distortion_coeff = {%g, %g, %g, %g},\n"
            % (distortion[0], distortion[1], distortion[2], distortion[3]),
            camera_text,
        )

    if replacements != 1:
        print("Failed to replace distortion.", file=sys.stderr)
        return True

    # Update the intrinsics
    prog = re.compile("intrinsic_matrix\s*=\s*\{.*?\}", re.DOTALL)
    intrinsic_string = (
        "intrinsic_matrix = {\n      %.8g, 0.0, %.8g,\n      0.0, %.8g, %.8g,\n      0.0, 0.0, 1.0\n    }"
        % (intrinsics[0], intrinsics[2], intrinsics[1], intrinsics[3])
    )
    (camera_text, replacements) = re.subn(prog, intrinsic_string, camera_text)
    if replacements != 1:
        print("Failed to replace intrinsics matrix.", file=sys.stderr)
        return True

    output_text = contents[:open_bracket] + camera_text + contents[close_bracket + 1 :]
    try:
        with open(filename, "w") as f:
            f.write(output_text)
    except IOError:
        print("Failed to open lua file " + filename, file=sys.stderr)
        return True
    return False


def calibrate_camera(
    bag,
    calibration_target,
    from_time,
    to_time,
    approx_sync,
    nav_cam_topic,
    haz_cam_topic,
    sci_cam_topic,
    dock_cam,
    depth_cam,
    sci_cam,
    only_depth_cam,
    verbose,
):

    fov_model = "pinhole-fov"
    radtan_model = "pinhole-radtan"

    HD_CAMERA_TOPIC = "/hw/cam_dock" if dock_cam else nav_cam_topic

    if HD_CAMERA_TOPIC == "/hw/cam_dock":
        DEPTH_CAMERA_TOPIC = "/hw/depth_perch/extended/amplitude_int"
    else:
        DEPTH_CAMERA_TOPIC = haz_cam_topic

    extra_flags = " --verbose" if verbose else " --dont-show-report"

    if from_time is not None and to_time is not None:
        extra_flags += " --bag-from-to " + from_time + " " + to_time

    if approx_sync is not None:
        extra_flags += " --approx-sync " + approx_sync

    if not os.path.isfile(bag):
        print("Bag file %s does not exist." % (bag), file=sys.stderr)
        return None
    if not os.path.isfile(calibration_target):
        print("Target file %s does not exist." % (calibration_target), file=sys.stderr)
        return None

    bag_dir = os.path.dirname(os.path.abspath(bag))
    bag_file = os.path.basename(bag)
    bag_name = os.path.splitext(bag_file)[0]

    if only_depth_cam:
        # Calibrate only the depth camera
        cmd = (
            "rosrun kalibr kalibr_calibrate_cameras --topics %s --models %s "
            + "--target %s --bag %s%s"
        ) % (
            DEPTH_CAMERA_TOPIC,
            radtan_model,
            calibration_target,
            bag_file,
            extra_flags,
        )
    elif sci_cam:
        # Calibrate the nav, haz, and sci cameras
        cmd = (
            "rosrun kalibr kalibr_calibrate_cameras --topics %s %s %s --models %s %s %s "
            + "--target %s --bag %s%s"
        ) % (
            HD_CAMERA_TOPIC,
            DEPTH_CAMERA_TOPIC,
            sci_cam_topic,
            fov_model,
            radtan_model,
            radtan_model,
            calibration_target,
            bag_file,
            extra_flags,
        )
    elif depth_cam:
        # Calibrate the HD and depth cameras
        cmd = (
            "rosrun kalibr kalibr_calibrate_cameras --topics %s %s --models %s %s "
            + "--target %s --bag %s%s"
        ) % (
            HD_CAMERA_TOPIC,
            DEPTH_CAMERA_TOPIC,
            fov_model,
            radtan_model,
            calibration_target,
            bag_file,
            extra_flags,
        )
    else:
        # Calibrate only the HD camera (nav or dock)
        cmd = (
            "rosrun kalibr kalibr_calibrate_cameras --topics %s "
            + "--models %s --target %s --bag %s%s"
        ) % (HD_CAMERA_TOPIC, fov_model, calibration_target, bag_file, extra_flags)

    print(("Running in: " + bag_dir))
    print(cmd)
    output_arg = None  # Display in the terminal what is going on
    ret = subprocess.call(
        cmd.split(), cwd=bag_dir, stdout=output_arg, stderr=output_arg
    )

    if ret != 0:
        print("Failed to calibrate camera from bag file.", file=sys.stderr)
        return None

    return (
        bag_dir + "/camchain-" + bag_name + ".yaml"
    )  # The file where kalibr writes its output


def main():
    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    default_calibration_target = SCRIPT_DIR + "/config/granite_april_tag.yaml"

    parser = argparse.ArgumentParser(description="Calibrate intrinsic parameters.")
    parser.add_argument(
        "robot",
        help="The name of the robot to configure (i.e., put p4d to edit p4d.config).",
    )
    parser.add_argument("bag", help="The bag file with calibration data.")
    parser.add_argument(
        "-d",
        "--dock_cam",
        dest="dock_cam",
        action="store_true",
        help="Calibrate dock camera.",
    )
    parser.add_argument(
        "-depth",
        "--depth_cam",
        dest="depth_cam",
        action="store_true",
        help="Calibrate respective depth camera.",
    )
    parser.add_argument(
        "--only_depth_cam",
        dest="only_depth_cam",
        action="store_true",
        help="Calibrate only the depth camera (haz or perch).",
    )
    parser.add_argument(
        "--sci_cam",
        dest="sci_cam",
        action="store_true",
        help="Calibrate nav_cam, haz_cam, and sci_cam.",
    )
    parser.add_argument(
        "-f",
        "--from",
        dest="from_time",
        help="Use the bag data starting at this time, in seconds.",
    )
    parser.add_argument(
        "-t",
        "--to",
        dest="to_time",
        help="Use the bag data until this time, in seconds.",
    )
    parser.add_argument(
        "--calibration_target",
        dest="calibration_target",
        help="Use this yaml file to desribe the calibration target, overriding the default: "
        + default_calibration_target,
    )
    parser.add_argument(
        "--approx_sync",
        dest="approx_sync",
        help="Time tolerance for approximate image synchronization [s] (default: 0.02).",
    )
    parser.add_argument(
        "--nav_cam_topic", default="/hw/cam_nav", help="The nav cam topic name."
    )
    parser.add_argument(
        "--haz_cam_topic",
        default="/hw/depth_haz/extended/amplitude_int",
        help="The haz cam topic name.",
    )
    parser.add_argument(
        "--sci_cam_topic", default="/hw/cam_sci", help="The sci cam topic name."
    )
    parser.add_argument(
        "-v", "--verbose", dest="verbose", action="store_true", help="Verbose mode."
    )
    args = parser.parse_args()

    if args.calibration_target is None:
        args.calibration_target = default_calibration_target

    # Sanity checks for sci_cam
    if args.sci_cam:
        if args.only_depth_cam or args.dock_cam:
            print(
                "The sci cam must be calibrated together with the nav and haz cams.",
                file=sys.stderr,
            )
            return
        if not args.depth_cam:
            print("Since sci cam is to be calibrated, also calibrate the haz cam.")
            args.depth_cam = True

    # Setup files
    yaml_file = calibrate_camera(
        args.bag,
        args.calibration_target,
        args.from_time,
        args.to_time,
        args.approx_sync,
        args.nav_cam_topic,
        args.haz_cam_topic,
        args.sci_cam_topic,
        args.dock_cam,
        args.depth_cam,
        args.sci_cam,
        args.only_depth_cam,
        args.verbose,
    )

    if yaml_file is None:
        print("Failed to run calibration.", file=sys.stderr)
        return

    print(("Reading calibration report file: " + yaml_file))

    if args.sci_cam:
        # The yaml file has info for nav, haz, and sci cameras
        (intrinsics, distortion, transforms) = read_yaml(
            yaml_file, ["cam0", "cam1", "cam2"]
        )
        if len(intrinsics) < 3 or len(distortion) < 3:
            print(
                "Failed to read intrinsics for the nav, haz, and sci cameras.",
                file=sys.stderr,
            )
            return
    elif args.depth_cam:
        # The yaml file has info for both cameras
        (intrinsics, distortion, transforms) = read_yaml(yaml_file, ["cam0", "cam1"])
        if len(intrinsics) < 2 or len(distortion) < 2:
            print("Failed to read depth camera parameters.", file=sys.stderr)
            return
    elif args.only_depth_cam:
        # The yaml file only has the depth camera info
        (intrinsics, distortion, transforms) = read_yaml(yaml_file, ["cam0"])
    else:
        # The yaml file has only the hd camera info
        (intrinsics, distortion, transforms) = read_yaml(yaml_file, ["cam0"])

    if not intrinsics or not distortion:
        print("Failed to read camera intrinsics.", file=sys.stderr)
        return

    config_file = SCRIPT_DIR + "/../../astrobee/config/robots/" + args.robot + ".config"
    print(("Updating intrinsics in file: " + config_file))

    if args.only_depth_cam:
        if update_intrinsics(
            config_file,
            "perch_cam" if args.dock_cam else "haz_cam",
            intrinsics[0],
            distortion[0],
        ):
            print(
                "Failed to update config file with depth camera intrinsics.",
                file=sys.stderr,
            )
            return
    else:
        # Replace the intrinsics of the HD camera
        if update_intrinsics(
            config_file,
            "dock_cam" if args.dock_cam else "nav_cam",
            intrinsics[0],
            distortion[0][0],
        ):
            print(
                "Failed to update config file with HD camera intrinsics.",
                file=sys.stderr,
            )
            return

        # Replace the intrinsics of the depth camera in addition to the HD camera
        if args.depth_cam:
            if update_intrinsics(
                config_file,
                "perch_cam" if args.dock_cam else "haz_cam",
                intrinsics[1],
                distortion[1],
            ):
                print(
                    "Failed to update config file with depth camera intrinsics.",
                    file=sys.stderr,
                )
                return

            # Update the haz cam to nav cam transform. This was not implemented for
            # the dock and perch cameras.
            if not args.dock_cam:
                trans = transforms[1]
                t = trans[0:3, 3]
                q = transformations.quaternion_from_matrix(trans)
                trans_name = "hazcam_to_navcam_transform"
                if lua_replace_transform(config_file, trans_name, (t, q)):
                    print(
                        "Will not update the value of "
                        + trans_name
                        + " (this one is needed only for the dense mapper).",
                        file=sys.stderr,
                    )
                    return

                # If the sci cam is present, also update sci cam intrinsics and the
                # sci cam to haz cam transform
                if args.sci_cam:

                    if update_intrinsics(
                        config_file, "sci_cam", intrinsics[2], distortion[2]
                    ):
                        print(
                            "Failed to update config file with sci camera intrinsics.",
                            file=sys.stderr,
                        )
                        return

                    trans = transforms[2]
                    t = trans[0:3, 3]
                    q = transformations.quaternion_from_matrix(trans)
                    trans_name = "scicam_to_hazcam_transform"
                    if lua_replace_transform(config_file, trans_name, (t, q)):
                        print(
                            "Will not update the value of "
                            + trans_name
                            + " (this one is needed only for the dense mapper).",
                            file=sys.stderr,
                        )
                        return


if __name__ == "__main__":
    main()
