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
Recursively copy a folder as "HTML symlinks" -- this means writing files
in the target folder with the same relative paths as in the source
folder, but instead of copying the file content, each output file is a
minimal HTML page that will redirect to the original file in the source
folder.
"""

import argparse
import os

HTML_TEMPLATE = """
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN">
<html><head><meta http-equiv=Refresh content="0;url={}"></head></html>
""".lstrip()


def copy_html_link(src_path_in, tgt_path_in, verbose):
    src_path = os.path.realpath(src_path_in)
    tgt_path = os.path.realpath(tgt_path_in)
    src_rel_tgt = os.path.relpath(src_path, tgt_path)
    count = 0
    for dir_path, dirs, files in os.walk(src_path):
        dir_path_suffix = dir_path.replace(src_path, "")
        dir_path_suffix = dir_path_suffix.lstrip("/")
        for f in files:
            if not dir_path_suffix:
                depth = 0
            else:
                depth = len(dir_path_suffix.split("/"))
            up_depth = "/".join([".."] * depth)
            src_f_rel_tgt_f = os.path.join(up_depth, src_rel_tgt, dir_path_suffix, f)
            out_path = os.path.join(tgt_path, dir_path_suffix, f)
            out_dir = os.path.dirname(out_path)
            if not os.path.isdir(out_dir):
                os.makedirs(out_dir, exist_ok=True)
            with open(out_path, "w") as out:
                out.write(HTML_TEMPLATE.format(src_f_rel_tgt_f))
            count += 1
            if verbose:
                print("%s -> %s" % (out_path, src_f_rel_tgt_f))
    print("wrote %s HTML redirect files" % count)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "src_path",
        type=str,
        help="source path",
    )
    parser.add_argument(
        "tgt_path",
        type=str,
        help="target path",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        default=False,
        help="make output more verbose",
    )

    args = parser.parse_args()

    copy_html_link(args.src_path, args.tgt_path, args.verbose)
