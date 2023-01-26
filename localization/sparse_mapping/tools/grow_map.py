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
import shutil
import subprocess
import sys


def check_bot():
    """It is very crucial that the bot is set."""
    bot = "ASTROBEE_ROBOT"
    if bot in os.environ:
        print((bot + "=" + os.environ[bot]))
    else:
        raise Exception("Must set " + bot)


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


def write_to_file(images, image_file):
    with open(image_file, "w+") as f:
        for image in images:
            f.write(image + "\n")


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


def extract_submap(input_map, output_map, image_file, work_dir):

    # image_file = work_dir + "/image_list.txt"
    # with open(image_file, 'w+') as f:
    #    for image in images:
    #        f.write(image + "\n")

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


def extract_submap_with_vocab_db(input_map, output_map, image_file):

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

    return bad_images


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
        "-small_map",
        type=str,
        required=True,
        help="Input registered, pruned, BRISK map with vocab db.",
    )
    parser.add_argument(
        "-big_map",
        type=str,
        required=True,
        help="Input registered, unpruned, BRISK map without vocab db.",
    )
    parser.add_argument(
        "-output_map",
        type=str,
        required=True,
        help="Output registered, pruned, BRISK map with vocab db.",
    )
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
        "-work_dir",
        type=str,
        required=True,
        help="Store all results in this directory.",
    )

    parser.add_argument("image_lists", nargs="*")

    args = parser.parse_args()

    return args


def add_neighbors_of_bad_images(all_images, bad_images):
    """Given a list of images in all_images, and a subset of them
    in bad_images, create a list of images that has all the images
    in bad_images, and for each such image also has the image
    before it and the image after it, as they show in all_images. The
    reason we want to add the neighbors for each bad image is that we
    will put all these in a map, and it is not good for localization
    that in a map an image shows up isolated.
    """

    all_images_dict = {}
    for image_count in range(len(all_images)):
        all_images_dict[all_images[image_count]] = image_count

    images_to_add_dict = {}
    for bad_image in bad_images:
        if bad_image not in all_images_dict:
            raise Exception("Book-keeping error.")
        image_count = all_images_dict[bad_image]

        # Add the bad image
        # print("add bad image " + bad_image)
        images_to_add_dict[image_count] = bad_image

        # Add its neighbors
        if image_count - 1 >= 0:
            # print("add neighbor " + all_images[image_count - 1])
            images_to_add_dict[image_count - 1] = all_images[image_count - 1]

        if image_count + 1 < len(all_images):
            # print("add neighbor " + all_images[image_count + 1])
            images_to_add_dict[image_count + 1] = all_images[image_count + 1]

    images_to_add = []
    for image_count in images_to_add_dict:
        # print("---add ", images_to_add_dict[image_count])
        images_to_add.append(images_to_add_dict[image_count])

    return images_to_add


def grow_map(args, curr_map_images, curr_map, source_map):
    """Images in source map that cannot be localized against curr_map are added to it.
    Addition is happening by assemblng all needed images and carving out the
    submap of args.big_map having them.
    """

    log_file = args.work_dir + "/bad_localization_list.txt"
    bad_images = find_hard_to_localize_images(args, curr_map, source_map, log_file)

    # Let the neigbhobs of bad (not localizable) images as also be bad
    source_map_images = list_images_in_map(source_map)

    all_bad_images = add_neighbors_of_bad_images(source_map_images, bad_images)

    # Update the curr map with the extra images. If no images to add, don't
    # change the current map
    if len(all_bad_images) > 0:

        # Add to the current images the bad images
        curr_map_images = sorted(set(curr_map_images + all_bad_images))

        curr_map_list = args.work_dir + "/curr_map_list.txt"
        print(("The grown map will use the images in: " + curr_map_list))
        write_to_file(curr_map_images, curr_map_list)

        curr_map = args.work_dir + "/grown_map.map"
        print(("Growing the map and building its vocab db. Writing it to: " + curr_map))
        extract_submap_with_vocab_db(args.big_map, curr_map, curr_map_list)

    else:
        print("No new images will be added at this stage.")

    return [curr_map_images, curr_map]


def main():

    args = parse_args()
    print(("Using histogram_equalization: " + str(args.histogram_equalization)))

    if not os.path.exists(args.small_map):
        raise Exception("The input small map does not exist.")
    if not os.path.exists(args.big_map):
        raise Exception("The input big map does not exist.")

    if (
        args.small_map == args.big_map
        or args.small_map == args.output_map
        or args.big_map == args.output_map
    ):
        raise Exception("Some of the specified maps have the same names.")

    check_bot()

    # TODO(oalexan1): Must ensure both maps are registered and that the second one
    # is larger than the first one!

    mkdir_p(args.work_dir)

    # These will be updated at each iteration
    curr_map = args.small_map
    curr_map_images = list_images_in_map(curr_map)

    big_map_images = list_images_in_map(args.big_map)

    for list_iter in range(len(args.image_lists)):

        # Will add images from this list
        candidate_list = args.image_lists[list_iter]

        # Sanity check
        missing_images = in_notin(curr_map_images, big_map_images)
        if len(missing_images) > 0:
            print(
                (
                    "The following images are in "
                    + args.big_map
                    + " but not in "
                    + curr_map
                )
            )
            print(missing_images)
            print("This violates the assumptions of this tool.")
            sys.exit(1)

        print(("See which images to add to map from: " + candidate_list))

        # Add those images for which localization against curr_map fails
        source_map = args.work_dir + "/" + "inc_map.map"
        extract_submap(args.big_map, source_map, candidate_list, args.work_dir)

        [curr_map_images, curr_map] = grow_map(
            args, curr_map_images, curr_map, source_map
        )

    # One last verification. Images in args.big_map that cannot be localized
    # against the map grown so far must be added to it.
    [curr_map_images, curr_map] = grow_map(
        args, curr_map_images, curr_map, args.big_map
    )

    if curr_map == args.small_map:
        print(
            (
                "The map did not grow. Create "
                + args.output_map
                + " as a copy of "
                + args.small_map
            )
        )
        shutil.copy(args.small_map, args.output_map)
    else:
        print(("Moving " + curr_map + " to " + args.output_map))
        shutil.move(curr_map, args.output_map)


if __name__ == "__main__":

    main()
