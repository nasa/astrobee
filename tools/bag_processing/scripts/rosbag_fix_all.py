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


def dosys(cmd):
    logging.info(cmd)
    ret = os.system(cmd)
    if ret != 0:
        logging.warning("Command failed with return value %s\n" % ret)
    return ret


def rosbag_fix_all(inbag_paths_in, jobs):
    this_folder = os.path.dirname(os.path.realpath(__file__))
    makefile = os.path.join(this_folder, "Makefile.rosbag_fix_all")
    inbag_paths = [p for p in inbag_paths_in if not p.endswith(".fix_all.bag")]
    skip_count = len(inbag_paths_in) - len(inbag_paths)
    if skip_count:
        logging.info(
            "Not trying to fix %d files that already end in .fix_all.bag", skip_count
        )
    outbag_paths = [os.path.splitext(p)[0] + ".fix_all.bag" for p in inbag_paths]
    outbag_paths_str = " ".join(outbag_paths)
    ret = dosys("make -f%s -j%s %s" % (makefile, jobs, outbag_paths_str))

    logging.info("")
    logging.info("====================")
    if ret == 0:
        logging.info("Fixed bags:")
        for outbag_path in outbag_paths:
            logging.info("  %s", outbag_path)
    else:
        logging.warning("Not all bags were fixed successfully (see errors above).")


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter,
):
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
    parser.add_argument("inbag", nargs="+", help="input bag")

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    rosbag_fix_all(args.inbag, args.jobs)

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)
