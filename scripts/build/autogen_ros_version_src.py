#!/usr/bin/env python
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

"""
Auto-generates / auto-selects files depending on whether --use-ros1 is
specified (ROS1) or not (ROS2).

Specifically, transforms a raw git checkout folder (default: "git_src")
to a ROS-version-specific workspace folder (default: "src") designed to
be used in a catkin (ROS1) or colcon (ROS2) workspace.

Example usage:
  cd ~/astrobee
  git clone https://github.com/nasa/astrobee.git git_src
  # Auto-generate ROS1 version of src directory. Normally this script is
  # called by configure.sh, but you could run it manually.
  ROS_VERSION=1 ./git_src/scripts/build/autogen_ros_version_src.py
  # ... continue with ROS1 build.
  cd ~/astrobee_ros2
  # ROS2 workspace can share git checkout with ROS1 workspace if desired
  ln -s ~/astrobee/git_src git_src
  # Auto-generate ROS2 version of src directory.
  ROS_VERSION=2 ./git_src/scripts/build/autogen_ros_version_src.py
  # ... continue with ROS2 build.

The transformation behavior is:
- The hierarchical folder structure of the checkout folder is copied to
  the autogen output folder. (The .git subfolder is skipped.)
- Plain files in the checkout folder are symlinked to the identical
  locations in the autogen output folder, but with the following
  transformations applied:
  - Any file that starts with the patterns "ros1-" or "ros2-" or contains
    the patterns ".ros1" or ".ros2" is (1) only symlinked if it the
    pattern matches the current ROS version, and (2) the symlinked
    filename has the pattern stripped out.
  - Any file that starts with the pattern "jinja-" or contains the
    pattern ".jinja" is interpreted as a Jinja2 template, which is
    rendered and output to the same filename with the pattern stripped
    out.  The Jinja2 rendering context contains the boolean variable
    ROS1 that can be used in Jinja2 conditionals to modify the content
    between ROS versions.
  - Any folder that contains a "ROS1_IGNORE" or "ROS2_IGNORE" marker
    file will not be propagated to the output folder if the name matches
    the current ROS version. Note that this behavior is similar to the
    ROS native "CATKIN_IGNORE" and "COLCON_IGNORE" marker files, but it
    turns out that both ROS versions respect the ignore markers of the
    other ROS version, which defeats our objective to have different
    behavior between the ROS versions.

This transformation has the following nice properties:
- It is performing an out-of-source build such that the checkout folder
  is not polluted with auto-generated files or auto-selected symlinks.
- The autogen output folder is not precious and can be automatically
  regenerated from the checkout folder at any time.
- A single checkout folder can be used with both ROS1 and ROS2
  workspaces, so you can make a modification once and test it in both
  workspaces before you push it.

This script is fancier than the bare minimum because it tries to make the
least possible changes to "src" each time it is run, so as not to disturb the
modification times of files (which could cause unnecessary recompilation).
"""

import argparse
import logging
import os
import shutil

import jinja2

EXPAND_TYPE = {
    "d": "directory",
    "l": "symlink",
    "f": "auto-generated file",
}

ABSOLUTE_SYMLINKS = True


def autogen_path(checkout_path, checkout_dir, autogen_dir):
    """
    Return the path under autogen_dir corresponding to the specified
    path under checkout_path.
    """
    return checkout_path.replace(checkout_dir, autogen_dir, 1)


def symlink_descriptor(src, dst):
    """
    Return a symlink descriptor pointing to src.
    """
    if ABSOLUTE_SYMLINKS:
        return {"type": "l", "target": src}
    else:
        return {"type": "l", "target": os.path.relpath(src, os.path.dirname(dst))}


def jinja_render(path, use_ros1):
    """
    Return the result of rendering the Jinja2 template file found at
    path. Set the ROS1 variable in the Jinja2 rendering context based
    on the value of use_ros1.
    """
    jinja_env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(os.path.dirname(path)),
        trim_blocks=True,
        lstrip_blocks=True,
    )
    tmpl = jinja_env.get_template(os.path.basename(path))
    return tmpl.render({"ROS1": use_ros1})


def get_current_state(autogen_dir):
    """
    Return the current state of autogen_dir, expressed as a dictionary
    mapping paths under autogen_dir to their current type and content.
    """
    result = {}
    for dir_path, dir_names, file_names in os.walk(autogen_dir, followlinks=False):
        result[dir_path] = {"type": "d"}

        for f in file_names:
            autogen_f = os.path.join(dir_path, f)

            if os.path.islink(autogen_f):
                result[autogen_f] = {"type": "l", "target": os.readlink(autogen_f)}
            else:
                with open(autogen_f, "r") as stream:
                    result[autogen_f] = {"type": "f", "content": stream.read()}

    return result


def strip_pattern(path, substr):
    """
    If fname either starts with the pattern "<substr>-" or contains the
    pattern ".<substr>", return fname with the pattern stripped
    out. Otherwise, return None.
    """
    d = os.path.dirname(path)
    f = os.path.basename(path)
    if f.startswith(substr + "-"):
        return os.path.join(d, f.replace(substr + "-", "", 1))
    elif ("." + substr) in f:
        return os.path.join(d, f.replace("." + substr, "", 1))
    else:
        return None


def get_desired_state(checkout_dir, autogen_dir, use_ros1):
    """
    Return the desired state of autogen_dir, expressed as a dictionary
    mapping paths under autogen_dir to their desired type and content.
    """
    result = {}
    ignore_marker = "ROS1_IGNORE" if use_ros1 else "ROS2_IGNORE"
    for dir_path, dir_names, file_names in os.walk(checkout_dir, followlinks=True):
        if ignore_marker in file_names:
            # if folder contains ignore_marker file, ignore it and all of its
            # children
            dir_names[:] = []
            continue

        if ".git" in dir_names:
            dir_names.remove(".git")  # skip any .git subfolder

        autogen_dir_path = autogen_path(dir_path, checkout_dir, autogen_dir)
        result[autogen_dir_path] = {"type": "d"}

        for f in file_names:
            checkout_f = os.path.join(dir_path, f)
            autogen_f = autogen_path(checkout_f, checkout_dir, autogen_dir)

            stripped_autogen_f = strip_pattern(autogen_f, "ros1")
            if stripped_autogen_f is not None:
                if use_ros1:
                    result[stripped_autogen_f] = symlink_descriptor(
                        checkout_f, stripped_autogen_f
                    )
                else:
                    pass  # don't symlink
                continue

            stripped_autogen_f = strip_pattern(autogen_f, "ros2")
            if stripped_autogen_f is not None:
                if use_ros1:
                    pass  # don't symlink
                else:
                    result[stripped_autogen_f] = symlink_descriptor(
                        checkout_f, stripped_autogen_f
                    )
                continue

            stripped_autogen_f = strip_pattern(autogen_f, "jinja")
            if stripped_autogen_f is not None:
                result[stripped_autogen_f] = {
                    "type": "f",
                    "content": jinja_render(checkout_f, use_ros1),
                }
                continue

            else:
                result[autogen_f] = symlink_descriptor(checkout_f, autogen_f)

    return result


def get_update_map(current_state, desired_state):
    """
    Generate an update map based on comparing the current and desired
    state. The update map is a dictionary mapping each path under "src"
    that requires an updates to a record containing that path's current
    and desired states, plus an description of the required update
    action.
    """
    autogen_paths = set(current_state.keys())
    autogen_paths.update(desired_state.keys())
    update_map = {
        p: {"current": current_state.get(p), "desired": desired_state.get(p)}
        for p in autogen_paths
    }
    update_map = {
        p: val for p, val in update_map.items() if val["current"] != val["desired"]
    }

    # annotate action description
    for p, val in update_map.items():
        current = val["current"]
        desired = val["desired"]
        if current is None:
            val["action"] = "add " + EXPAND_TYPE[desired["type"]]
        elif desired is None:
            val["action"] = "delete " + EXPAND_TYPE[current["type"]]
        else:
            val["action"] = "update " + EXPAND_TYPE[desired["type"]]

    return update_map


def apply_update_map(update_map, dry_run):
    """
    Apply the required updates in the update map. If dry_run is True,
    generate the usual debug output but don't actually apply any
    changes.
    """
    for p in sorted(update_map.keys()):
        val = update_map[p]
        logging.debug("%s: %s", p, val["action"])
        if dry_run:
            continue

        current = val["current"]
        desired = val["desired"]

        if current is not None:
            if os.path.exists(p):
                if current["type"] == "d":
                    shutil.rmtree(p)
                else:
                    os.unlink(p)

        if desired is not None:
            if not os.path.exists(os.path.dirname(p)):
                os.makedirs(os.path.dirname(p))

            if desired["type"] == "d":
                os.mkdir(p)
            elif desired["type"] == "l":
                os.symlink(desired["target"], p)
            else:
                with open(p, "w") as out:
                    out.write(desired["content"])

    if dry_run:
        logging.info("[dry run] would have applied %s updates", len(update_map))
    else:
        logging.info("applied %s updates", len(update_map))


def autogen_ros_version_src(use_ros1, dry_run, checkout_dir, autogen_dir):
    checkout_dir = os.path.realpath(checkout_dir)
    autogen_dir = os.path.realpath(autogen_dir)

    logging.warning("generating %s directory from %s..." % (autogen_dir, checkout_dir))
    current_state = get_current_state(autogen_dir)
    desired_state = get_desired_state(checkout_dir, autogen_dir, use_ros1)
    update_map = get_update_map(current_state, desired_state)
    apply_update_map(update_map, dry_run)
    logging.warning(
        "generating %s directory from %s... done" % (autogen_dir, checkout_dir)
    )


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-v",
        "--verbose",
        help="make output more verbose (can specify multiple times)",
        action="append_const",
        const=1,
        default=[],
    )
    parser.add_argument(
        "-r",
        "--ros-version",
        help="override ROS_VERSION environment variable (specify '1' or '2')",
        default=os.getenv("ROS_VERSION"),
    )
    parser.add_argument(
        "-d",
        "--dry-run",
        help="don't actually apply updates",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-c",
        "--checkout-dir",
        help="specify location of git source checkout (must be outside <workspace>/src)",
        default="git_src",
    )
    parser.add_argument(
        "-a",
        "--autogen-dir",
        help="specify where to write autogen output (<workspace>/src or a subfolder)",
        default="src",
    )

    args = parser.parse_args()

    if not args.ros_version in ("1", "2"):
        parser.error(
            "ROS_VERSION environment variable or --ros-version arg must be specified with value '1' or '2'"
        )
    if not os.path.isdir(args.checkout_dir):
        parser.error("checkout dir %s is not a directory" % args.checkout_dir)
    if os.path.exists(os.path.join(args.autogen_dir, ".git")):
        parser.error(
            "autogen output dir %s must not be a git checkout!" % args.autogen_dir
        )

    level = logging.WARNING - (10 * sum(args.verbose))
    logging.basicConfig(level=level, format="%(message)s")
    autogen_ros_version_src(
        args.ros_version == "1", args.dry_run, args.checkout_dir, args.autogen_dir
    )


if __name__ == "__main__":
    main()
