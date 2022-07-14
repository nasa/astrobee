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

import argparse
import os
import re
import subprocess
import sys

from hsi import *  # load hugin


def run_cmd(cmd, logfile=""):

    print((" ".join(cmd)))

    if logfile != "":
        f = open(logfile, "w")

    stdout = ""
    stderr = ""
    popen = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True
    )
    for stdout_line in iter(popen.stdout.readline, ""):
        if logfile != "":
            f.write(stdout_line)

        stdout += stdout_line

    popen.stdout.close()
    return_code = popen.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)

    if return_code != 0:
        print(("Failed to run command.\nOutput: ", stdout, "\nError: ", stderr))

    return (return_code, stdout, stderr)


def list_images_in_map(mapfile):

    cmd = ["rosrun", "sparse_mapping", "build_map", "-info", "-output_map", mapfile]

    (returncode, stdout, stderr) = run_cmd(cmd)

    if returncode != 0:
        print(("Failed to run: " + " ".join(cmd)))

    images = []
    for line in (stdout + "\n" + stderr).split("\n"):

        # Print warning messages to stdout
        m = re.match("^.*?\s([^\s]*?jpg)", line)
        if m:
            images.append(m.group(1))

    return images


def parse_args():

    parser = argparse.ArgumentParser(description="Generates/updates hugin files.")
    parser.add_argument(
        "-map_name",
        type=str,
        required=True,
        help="Input surf map.",
    )
    parser.add_argument(
        "-input_hugin",
        type=str,
        required=False,
        help="Input Hugin pto file.",
    )
    parser.add_argument(
        "-output_hugin",
        type=str,
        required=False,
        help="Output Hugin pto file.",
    )
    parser.add_argument(
        "-work_dir",
        type=str,
        required=False,
        help="Store all results in this directory.",
    )

    parser.add_argument("image_lists", nargs="*")

    args = parser.parse_args()

    return args


def main():

    args = parse_args()
    source_map_images = list_images_in_map(args.map_name)

    p = Panorama()  # make a new Panorama object

    if args.output_hugin is None:
        output_hugin = args.map_name.replace("_surf.map", ".pto")
    else:
        output_hugin = args.output_hugin

    if args.input_hugin is not None:
        # create a C++ std::ifstream
        ifs = ifstream(args.input_hugin)
        # read the pto file into the Panorama object
        p.readData(ifs)
        # don't need anymore
        del ifs

        # Delete existing image files from the to-add list
        for x in range(0, p.getNrOfImages()):
            img = p.getImage(x)
            name = img.getFilename()
            if name in source_map_images:
                source_map_images.remove(name)

    for img in source_map_images:
        srcImage = SrcPanoImage()
        srcImage.setFilename(img)
        p.addImage(srcImage)

    # make a c++ std::ofstream to write to
    ofs = ofstream(output_hugin)
    # write the modified panorama to that stream
    p.writeData(ofs)
    # done with it
    del ofs


if __name__ == "__main__":

    main()
