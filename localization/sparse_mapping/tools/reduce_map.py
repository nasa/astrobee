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

# Create a small map out of a big map that has the same predictive
# power. That is tested by localizing the images from the big map
# against the created small map. See the documentation for more info.

import argparse
import os
import random
import re
import subprocess
import sys


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


def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError:
        if os.path.isdir(path):
            pass
        else:
            raise Exception(
                "Could not make directory " + path + " as a file with this name exists."
            )


def read_image_list(filename):
    images = []
    with open(filename, "r") as f:
        for image in f.read().splitlines():
            images.append(image.rstrip())

    if len(images) == 0:
        print(("No images were read from: " + filename))
        sys.exit(1)

    return images


def list_images_in_map(mapfile):

    cmd = ["build_map", "-info", "-output_map", mapfile]

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


def extract_submap_with_vocab_db(input_map, output_map, images, work_dir):

    image_file = work_dir + "/image_list.txt"
    with open(image_file, "w+") as f:
        for image in images:
            f.write(image + "\n")

    print(("Extracting submap from " + input_map + " to " + output_map))
    cmd = [
        "extract_submap",
        "-input_map",
        input_map,
        "-output_map",
        output_map,
        "-skip_bundle_adjustment",
    ]

    cmd += ["-image_list", image_file]

    run_cmd(cmd)

    print(("Building vocab db and pruning " + output_map))
    cmd = ["build_map", "-output_map", output_map, "-vocab_db"]
    run_cmd(cmd)


def parse_localization_log_str(log_str):
    """From:
    Errors for mydir/009151.jpg: 1e+06 m
    extract the image name and the error value.
    """

    images = []
    errors = []
    for line in log_str.split("\n"):
        m = re.match("^Errors\s+for\s+(.*?.jpg):\s*(.*?)\s", line)
        if not m:
            continue
        image = m.group(1)
        error = float(m.group(2))
        images.append(image)
        errors.append(error)

    return (images, errors)


def parse_localization_log_file(log_file):
    with open(log_file, "r") as f:
        all_data = f.read()
    return parse_localization_log_str(all_data)


def parse_images_with_bad_localization(log_str, localization_error):

    """Return the images for which error value is > localization_error."""

    bad_images_set = set()
    bad_images = []
    errors = []
    [all_imags, all_errors] = parse_localization_log_str(log_str)

    for it in range(len(all_imags)):
        image = all_imags[it]
        error = all_errors[it]
        if error > localization_error:
            bad_images_set.add(image)
            bad_images.append(image)
            errors.append(error)

    return (bad_images_set, bad_images, errors)


def find_hard_to_localize_images(args, ref_map, source_map, out_file):

    # See if the remaining images are enough to localize images in the full map
    cmd = [
        "localize_cams",
        "-reference_map",
        ref_map,
        "-source_map",
        source_map,
        "-min_brisk_threshold",
        str(args.min_brisk_threshold),
        "-default_brisk_threshold",
        str(args.default_brisk_threshold),
        "-max_brisk_threshold",
        str(args.max_brisk_threshold),
        "-early_break_landmarks",
        str(args.early_break_landmarks),
        "-num_similar",
        str(args.num_similar),
        "-ransac_inlier_tolerance",
        str(args.ransac_inlier_tolerance),
        "-num_ransac_iterations",
        str(args.num_ransac_iterations),
    ]
    if args.histogram_equalization:
        cmd.append("-histogram_equalization")

    full_log = out_file + ".log"
    print(("Writing the log of localization to: " + full_log))

    (returncode, stdout, stderr) = run_cmd(cmd, full_log)

    (bad_images_set, bad_images, errors) = parse_images_with_bad_localization(
        stdout + "\n" + stderr, args.localization_error
    )
    print(
        (
            "Number of images with localization error > "
            + str(args.localization_error)
            + " with reference map: "
            + ref_map
            + " is: "
            + str(len(bad_images))
        )
    )

    print(("Writing these images to: " + out_file))
    with open(out_file, "w+") as f:
        f.write("# image and localization error:\n")
        for count in range(len(bad_images)):
            f.write(bad_images[count] + " " + str(errors[count]) + "\n")

    return bad_images_set


def check_subset(list1, list2):
    """Check that all images in the first list are also present in the second."""

    set2 = set()
    for val in list2:
        set2.add(val)

    for val in list1:
        if val not in set2:
            print(("Missing image " + str(val)))
            return False

    return True


def in_notin(list1, list2):
    """Find entries in the first list that are not in the second list."""

    set2 = set()
    for val in list2:
        set2.add(val)

    out_list = []
    for val in list1:
        if val not in set2:
            out_list.append(val)

    return out_list


def parse_args():

    parser = argparse.ArgumentParser(
        description="Remove images from a map that appear reduntant."
    )
    parser.add_argument(
        "-input_map",
        type=str,
        required=True,
        help="Input registered, unpruned, BRISK map with vocab db.",
    )
    # parser.add_argument('-output_map', type = str, required = True,
    #                    help='Output registered, pruned, BRISK map with vocab db.')
    parser.add_argument(
        "-min_brisk_threshold",
        type=int,
        required=False,
        default=20,
        help="Min BRISK threshold.",
    )
    parser.add_argument(
        "-default_brisk_threshold",
        type=int,
        required=False,
        default=90,
        help="Default BRISK threshold.",
    )
    parser.add_argument(
        "-max_brisk_threshold",
        type=int,
        required=False,
        default=110,
        help="Max BRISK threshold.",
    )
    parser.add_argument(
        "-early_break_landmarks",
        type=int,
        required=False,
        default=100,
        help="Break early when we have this many landmarks during localization.",
    )
    parser.add_argument(
        "-histogram_equalization",
        dest="histogram_equalization",
        action="store_true",
        required=False,
    )

    parser.add_argument(
        "-num_similar",
        type=int,
        required=False,
        default=20,
        help="Use in localization this many images which "
        + "are most similar to the image to localize.",
    )
    parser.add_argument(
        "-num_ransac_iterations",
        type=int,
        required=False,
        default=100,
        help="Use in localization this many ransac iterations.",
    )
    parser.add_argument(
        "-ransac_inlier_tolerance",
        type=int,
        required=False,
        default=5,
        help="Use in localization this inlier tolerance.",
    )
    parser.add_argument(
        "-sample_rate",
        type=float,
        required=False,
        default=0.25,
        help="The fraction of images to try to remove from the map at once.",
    )
    parser.add_argument(
        "-localization_error",
        type=float,
        required=False,
        default=0.02,
        help="An image that has localization error bigger than this, "
        + "in meters, is considered hard to localize.",
    )
    parser.add_argument(
        "-big_localization_error",
        type=float,
        required=False,
        default=0.05,
        help="Image whose localization error went over this threshold as "
        + "result of reducing the map should be flagged.",
    )
    parser.add_argument(
        "-attempts",
        type=int,
        required=False,
        default=2,
        help="How many times to try to reduce the map.",
    )
    parser.add_argument(
        "-work_dir",
        type=str,
        required=True,
        help="Store all results in this directory.",
    )
    parser.add_argument(
        "-image_list",
        type=str,
        required=False,
        help="Instead of taking out images of the map randomly, start with a submap with the images from this list.",
    )
    # parser.add_argument("-v", "-verbosity", type=int,
    # help="increase output verbosity")
    args = parser.parse_args()

    return args


def main():

    args = parse_args()

    mkdir_p(args.work_dir)

    print(("Using histogram_equalization: " + str(args.histogram_equalization)))

    input_list = []
    if args.image_list is not None:
        print(("Initalizing the reduced map with images from " + args.image_list))
        args.attempts = 1
        print("Using just one attempt, hence only trying to add to the input list.")
        input_list = read_image_list(args.image_list)
    else:
        print(("Using " + str(args.attempts) + " at reducing the map."))

    prev_map = args.input_map
    for outer_iter in range(args.attempts):

        print("\n\n-------------------------------------------")
        print(("Attempting to remove images. Iteration: " + str(outer_iter) + "."))

        images = list_images_in_map(prev_map)
        print(("Current iteration map has " + str(len(images)) + " images."))

        if args.image_list is None:
            exclude_images = random.sample(images, int(len(images) * args.sample_rate))
            print(
                (
                    "Randomly excluding "
                    + str(len(exclude_images))
                    + " images from the map."
                )
            )
        else:
            if not check_subset(input_list, images):
                print(
                    (
                        "The images in "
                        + args.image_list
                        + " must be all in the input map: "
                        + args.input_map
                    )
                )
                sys.exit(1)

            exclude_images = in_notin(images, input_list)
            print(
                (
                    "Start by excluding "
                    + str(len(exclude_images))
                    + " images from the map."
                )
            )

        # for image in images:
        #    print("image is ", image)
        #
        # print("subimages: ", exclude_images)
        # print("size is ", len(exclude_images))

        reduced_log_file = ""
        submap = ""
        include_images = []

        for inner_iter in range(10):

            print(("\nAdding images back. Iteration: " + str(inner_iter) + "."))

            submap = args.work_dir + "/" + "submap_iter" + str(outer_iter) + ".map"

            include_images = in_notin(images, exclude_images)

            extract_submap_with_vocab_db(
                args.input_map, submap, include_images, args.work_dir
            )

            reduced_log_file = (
                args.work_dir
                + "/bad_localization_with_reduced_map_outer_iter"
                + str(outer_iter)
                + "_inner_iter_"
                + str(inner_iter)
                + ".txt"
            )
            bad_images_set = find_hard_to_localize_images(
                args, submap, args.input_map, reduced_log_file
            )

            # Put back the images that we can't take out as they have bad localization error
            new_exclude_images = []
            num_added_back = 0
            for image in exclude_images:
                if image in bad_images_set:
                    num_added_back += 1
                else:
                    new_exclude_images.append(image)
            exclude_images = new_exclude_images[:]
            print(("Added back to the map " + str(num_added_back) + " image(s)."))

            if num_added_back == 0:
                print("No more images to add back to the map. Stopping.")
                break

        # Do some stats
        (reduced_images, reduced_errors) = parse_localization_log_file(reduced_log_file)

        print(
            (
                "Images for which localization error is larger than "
                + str(args.localization_error)
                + " are:"
            )
        )
        for local_it in range(len(reduced_images)):
            if reduced_errors[local_it] > args.localization_error:
                print((reduced_images[local_it] + " " + str(reduced_errors[local_it])))

        print(
            (
                "The reduced map for attempt "
                + str(outer_iter)
                + " is saved to "
                + submap
            )
        )
        print(("It has " + str(len(include_images)) + " images."))

        prev_map = submap


if __name__ == "__main__":

    main()
