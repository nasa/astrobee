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
Master script to apply all passes of processing needed to fix our legacy bag
files. The actual processing steps are found in Makefile.rosbag_fix_all.
"""

from __future__ import print_function

import argparse
import logging
import os
import sys


def dosys(cmd):
    logging.info(cmd)
    ret = os.system(cmd)
    if ret != 0:
        logging.warning("Command failed with return value %s\n" % ret)
    return ret


def rosbag_fix_all(
    inbag_paths_in, jobs, debayer, decode_haz, filter_args, deserialize=False
):
    this_folder = os.path.dirname(os.path.realpath(__file__))
    makefile = os.path.join(this_folder, "Makefile.rosbag_fix_all")
    inbag_paths_in = [p for p in inbag_paths_in if p.endswith(".bag")]

    # Skip processed bags to start in the correct stage in the Makefile
    inbag_paths = [p for p in inbag_paths_in if not ".rewrite_types.bag" in p]
    inbag_paths = [p for p in inbag_paths if not ".migrate_old.bag" in p]
    inbag_paths = [p for p in inbag_paths if not ".debayer.bag" in p]
    inbag_paths = [p for p in inbag_paths if not ".depth_split.bag" in p]
    inbag_paths = [p for p in inbag_paths if not ".fix_all.bag" in p]

    skip_count = len(inbag_paths_in) - len(inbag_paths)
    if skip_count:
        logging.info(
            "Not trying to fix %d files that already end in .fix_all.bag", skip_count
        )

    outbag_paths = [os.path.splitext(p)[0] + ".fix_all.bag" for p in inbag_paths]
    outbag_paths_str = " ".join(outbag_paths)

    # Rosbag migrate message definitions
    rewrite_types_args = "-v"

    # Rosbag verify bags
    rosbag_verify_args = "-v"
    if deserialize:
        rosbag_verify_args += " -d"

    # Rosbag convert debayer
    if debayer:
        rosbag_debayer_args = "-s"
    else:
        rosbag_debayer_args = "-n"

    # Rosbag decode haz cam
    if decode_haz == "":
        rosbag_pico_split_extended_args = "-n"
    else:
        output_stream = os.popen("catkin_find --first-only bag_processing resources")
        coeff_path = (
            output_stream.read().rstrip() + "/" + decode_haz + "_haz_xyz_coeff.npy"
        )
        # Assert that coefficient file exists
        if os.path.exists(coeff_path):
            rosbag_pico_split_extended_args = "-s --in_npy " + coeff_path
        else:
            logging.warning(
                "Depth cam robot coefficient file " + coeff_path + " does not exist."
            )
            return 1

    # Rosbag filter
    rosbag_filter_args = filter_args

    # Call entire pipeline on all the files
    # "1>&2": redirect stdout to stderr to see make's command echo in rostest output
    ret1 = dosys(
        'REWRITE_TYPES_ARGS="%s" ROSBAG_VERIFY_ARGS="%s" ROSBAG_DEBAYER_ARGS="%s" ROSBAG_SPLIT_DEPTH_ARGS="%s" ROSBAG_FILTER_ARGS="%s" make -f%s -j%s %s 1>&2'
        % (
            rewrite_types_args,
            rosbag_verify_args,
            rosbag_debayer_args,
            rosbag_pico_split_extended_args,
            rosbag_filter_args,
            makefile,
            jobs,
            outbag_paths_str,
        )
    )

    logging.info("")
    logging.info("====================")
    if ret1 == 0:
        logging.info("Fixed bags:")
        for outbag_path in outbag_paths:
            logging.info("  %s", outbag_path)
    else:
        logging.warning("Not all bags were fixed successfully (see errors above).")
        logging.warning(
            "You can debug any failed output bags - ending in .fix_all_pre_check.bag"
        )
        logging.warning(
            "If you want to try again, clean first: rm *.fix_all_pre_check.bag"
        )
        return ret1

    # Merge resulting bags
    inbag_folders_in = list(set([os.path.split(p)[0] for p in inbag_paths_in]))
    for inbag_folder_in in inbag_folders_in:
        output_stream = os.popen(
            "catkin_find --first-only bag_processing scripts/rosbag_merge.py"
        )
        if inbag_folder_in == "":
            path = ""
        else:
            path = " -d " + inbag_folder_in
        merge_bags_path = (
            output_stream.read().rstrip() + path + " --input-bag-suffix .fix_all.bag"
        )
        ret2 = dosys(merge_bags_path)

    return ret2


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter):
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-j",
        "--jobs",
        help="specifies the number of jobs to run simultaneously",
        type=int,
        default=1,
    )
    parser.add_argument(
        "-d",
        "--deserialize",
        help="perform deserialization check on output bag (can be slow for large bags)",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "--debayer",
        help="debayer the nav or dock cam",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "--decode-haz",
        help="decode the extended haz cam topic into points and amplitude_int, argument is robot name",
        default="",
        type=str,
    )
    parser.add_argument(
        "--filter",
        help='filter the bagfile. Use with "" quotes. Example:  --filter "--accept /loc/* --reject /loc/ml/features"',
        default="",
        type=str,
    )
    parser.add_argument("inbag", nargs="+", help="input bag")

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    ret = rosbag_fix_all(
        args.inbag,
        args.jobs,
        args.debayer,
        args.decode_haz,
        args.filter,
        deserialize=args.deserialize,
    )

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)

    sys.exit(0 if ret == 0 else 1)
