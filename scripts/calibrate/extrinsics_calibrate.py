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


# returns extrinsics (T_cam_imu) from the yaml file
def read_yaml_extrinsics(filename, cameras):
    extrinsics = []
    with open(filename, "r") as f:
        try:
            d = yaml.load(f)
            for cam in cameras:
                extrinsics.append(np.array(d[cam]["T_cam_imu"]))
        except yaml.YAMLError as exc:
            print(exc, file=sys.stderr)
    return extrinsics


def has_depth_topic(filename):
    """See if a depth topic is in the file."""
    with open(filename, "r") as f:
        for line in f:
            prog = re.compile("^\s*rostopic:\s*/hw/depth")
            m = re.match(prog, line)
            if m:
                return True

    return False


def has_dock_topic(filename):
    """See if a dock topic is in the file."""
    with open(filename, "r") as f:
        for line in f:
            prog = re.compile("^\s*rostopic:\s*/hw/.*?dock")
            m = re.match(prog, line)
            if m:
                return True

    return False


def calibrate_extrinsics(
    bag,
    calibration_target,
    intrinsics_yaml,
    imu_yaml,
    from_time,
    to_time,
    padding,
    verbose=False,
):

    extra_flags = " --verbose" if verbose else " --dont-show-report"

    if from_time is not None and to_time is not None:
        extra_flags += " --bag-from-to " + from_time + " " + to_time

    if padding is not None:
        extra_flags += " --timeoffset-padding " + padding

    if not os.path.isfile(bag):
        print("Bag file %s does not exist." % (bag), file=sys.stderr)
        return None
    if not os.path.isfile(calibration_target):
        print("Target file %s does not exist." % (calibration_target), file=sys.stderr)
        return None
    if not os.path.isfile(intrinsics_yaml):
        print("Intrinsics file %s does not exist." % (intrinsics_yaml), file=sys.stderr)
        return None
    if not os.path.isfile(imu_yaml):
        print("IMU file %s does not exist." % (imu_yaml), file=sys.stderr)
        return None

    bag_dir = os.path.dirname(os.path.abspath(bag))
    bag_file = os.path.basename(bag)
    bag_name = os.path.splitext(bag_file)[0]

    # Display in the terminal what is going on
    output_arg = None

    cmd = (
        "rosrun kalibr kalibr_calibrate_imu_camera --bag %s --cam %s --imu %s "
        + "--target %s --time-calibration%s"
    ) % (bag_file, intrinsics_yaml, imu_yaml, calibration_target, extra_flags)

    print(("Running in: " + bag_dir))
    print(cmd)
    ret = subprocess.call(
        cmd.split(), cwd=bag_dir, stdout=output_arg, stderr=output_arg
    )
    if ret != 0:
        print("Failed to calibrate extrinsics from bag file.", file=sys.stderr)
        return None
    return bag_dir + "/camchain-imucam-" + bag_name + ".yaml"


def trans_quat_to_transform(xxx_todo_changeme):
    (trans, quat) = xxx_todo_changeme
    if trans is None or quat is None:
        return None
    # I think Jesse is confused with what is actually stored in the file
    # because his quaternion_to_dcm seems to give the conjugate of the right quaternion
    m = transformations.quaternion_matrix(transformations.quaternion_conjugate(quat))
    m[0, 3] = -trans[0]
    m[1, 3] = -trans[1]
    m[2, 3] = -trans[2]
    return m


def has_two_cameras(filename):
    try:
        with open(filename, "r") as f:
            contents = f.read()
    except IOError:
        print("Failed to open file: " + filename, file=sys.stderr)
        return False

    prog = re.compile("cam1")
    match = re.search(prog, contents)
    if match:
        return True
    else:
        return False


def main():

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    default_calibration_target = SCRIPT_DIR + "/config/granite_april_tag.yaml"

    parser = argparse.ArgumentParser(description="Calibrate extrinsics parameters.")
    parser.add_argument(
        "robot",
        help="The name of the robot to configure (i.e., put p4d to edit p4d.config).",
    )
    parser.add_argument(
        "intrinsics_yaml", help="The yaml file with intrinsics calibration data."
    )
    parser.add_argument(
        "extrinsics_bag", help="The bag file with extrinsics calibration data."
    )
    parser.add_argument(
        "-d",
        "--dock_cam",
        dest="dock_cam",
        action="store_true",
        help="Calibrate dock camera.",
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
        "--timeoffset-padding",
        dest="padding",
        help="Maximum range in which the timeoffset may change during estimation, in seconds. See readme.md for more info. (default: 0.01)",
    )
    parser.add_argument(
        "--calibration_target",
        dest="calibration_target",
        help="Use this yaml file to desribe the calibration target, overriding the default: "
        + default_calibration_target,
    )
    parser.add_argument(
        "-v", "--verbose", dest="verbose", action="store_true", help="Verbose mode."
    )
    args = parser.parse_args()

    if args.calibration_target is None:
        args.calibration_target = default_calibration_target

    imu_yaml = SCRIPT_DIR + "/config/imu.yaml"
    config_file = SCRIPT_DIR + "/../../astrobee/config/robots/" + args.robot + ".config"

    # Sanity check
    has_dock = has_dock_topic(args.intrinsics_yaml)
    if has_dock and (not args.dock_cam):
        print(
            "File "
            + args.intrinsics_yaml
            + " + has a dock topic, "
            + " yet the --dock-cam flag was not used.",
            file=sys.stderr,
        )
        return

    print("Calibrating camera extrinsics...")
    extrinsics_yaml = calibrate_extrinsics(
        args.extrinsics_bag,
        args.calibration_target,
        args.intrinsics_yaml,
        imu_yaml,
        args.from_time,
        args.to_time,
        args.padding,
        verbose=args.verbose,
    )

    if extrinsics_yaml == None:
        print("Failed to calibrate extrinsics.", file=sys.stderr)
        return

    two_cams = has_two_cameras(extrinsics_yaml)
    if two_cams:
        imu_to_camera = read_yaml_extrinsics(extrinsics_yaml, ["cam0", "cam1"])
    else:
        imu_to_camera = read_yaml_extrinsics(extrinsics_yaml, ["cam0"])

    if not imu_to_camera:
        print("Failed to read extrinsics from yaml file.", file=sys.stderr)
        return

    body_to_imu = trans_quat_to_transform(
        lua_read_transform(config_file, "imu_transform")
    )
    if body_to_imu is None:
        print("Failed to read imu transform.", file=sys.stderr)
        return

    imu_to_body = np.linalg.inv(body_to_imu)
    body_to_camera = np.linalg.inv(imu_to_body.dot(np.linalg.inv(imu_to_camera[0])))

    q = transformations.quaternion_conjugate(
        transformations.quaternion_from_matrix(body_to_camera)
    )
    t = imu_to_body.dot(np.linalg.inv(imu_to_camera[0])[0:4, 3])
    print("Cam0:")
    print("Translation: ", t)
    print("Rotation quaternion: ", q)

    if two_cams:
        # Modify the transform for the second camera, that is the depth one
        body_to_depth_camera = np.linalg.inv(
            imu_to_body.dot(np.linalg.inv(imu_to_camera[1]))
        )
        q_depth = transformations.quaternion_conjugate(
            transformations.quaternion_from_matrix(body_to_depth_camera)
        )
        t_depth = imu_to_body.dot(np.linalg.inv(imu_to_camera[1])[0:4, 3])
        print("Cam1:")
        print("Translation: ", t_depth)
        print("Rotation quaternion: ", q_depth)
        if lua_replace_transform(
            config_file,
            "perch_cam_transform" if args.dock_cam else "haz_cam_transform",
            (t_depth, q_depth),
        ):
            print(
                "Failed to update config file with depth camera parameters.",
                file=sys.stderr,
            )
            return

    has_depth = has_depth_topic(args.intrinsics_yaml)
    if has_depth and (not two_cams):
        # Only the depth camera is present, so the first transform corresponds to the depth camera
        if lua_replace_transform(
            config_file,
            "perch_cam_transform" if args.dock_cam else "haz_cam_transform",
            (t, q),
        ):
            print(
                "Failed to update config file with depth camera parameters",
                file=sys.stderr,
            )
            return
    else:
        # Either both cameras are present, or only the HD cam, so the first transform
        # corresponds to the HD camera
        if lua_replace_transform(
            config_file,
            "dock_cam_transform" if args.dock_cam else "nav_cam_transform",
            (t, q),
        ):
            print(
                "Failed to update config file with HD camera parameters",
                file=sys.stderr,
            )
            return


if __name__ == "__main__":
    main()
