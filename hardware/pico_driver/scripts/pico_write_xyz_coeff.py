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
Given a bag containing Pico Flexx points messages, output a *.npy file
containing array xyz coefficients that can subsequently be used to
reconstruct point clouds from Pico Flexx extended messages.
"""

import argparse
import logging

import numpy as np
import pico_utils as pico


def pico_write_xyz_coeff(
    in_bag_path, out_npy_path, fast=False, verbose=False, cam=pico.DEFAULT_CAM
):
    xyz_coeff = pico.get_xyz_coeff(in_bag_path, fast=fast, cam=cam)
    with open(out_npy_path, "wb") as out_npy:
        np.save(out_npy, xyz_coeff, allow_pickle=False)
    logging.info("wrote pico xyz coefficients to %s", out_npy_path)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="print more verbose debug info",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-f",
        "--fast",
        help="speed up testing by only processing part of the bag",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-c",
        "--cam",
        help="specify camera for rosbag topic filtering [%(default)s]",
        nargs="?",
        choices=pico.CAM_CHOICES,
        default=pico.DEFAULT_CAM,
    )
    parser.add_argument("in_bag", help="input bag containing points messages")
    parser.add_argument("out_npy", help="output npy file of xyz coefficients")

    args = parser.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    pico_write_xyz_coeff(
        args.in_bag, args.out_npy, fast=args.fast, verbose=args.verbose, cam=args.cam
    )

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)


if __name__ == "__main__":
    main()
