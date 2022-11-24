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
Specifically, transforms a raw git checkout ".astrobee" folder to a
ROS-version-specific "src" folder designed to be used in a catkin (ROS1)
or colcon (ROS2) workspace. The git checkout needs to be a hidden folder,
otherwise its contents will clash with "src" during colcon build.
Example usage:
  cd ~/astrobee
  git clone https://github.com/nasa/astrobee.git .astrobee
  # Auto-generate ROS1 version of src directory. Normally this script is
  # called by configure.sh, but you could run it manually.
  ./.astrobee/scripts/build/autogen_ros_version_src.py --use-ros1
  # ... continue with ROS1 build.
  cd ~/astrobee_ros2
  # ROS2 workspace can share git checkout with ROS1 workspace if desired
  ln -s ~/astrobee/.astrobee .astrobee
  # Auto-generate ROS2 version of src directory. ROS2 is the default if
  # --use-ros1 is not specified.
  ./.astrobee/scripts/build/autogen_ros_version_src.py
  # ... continue with ROS2 build.
The transformation behavior is:
- The hierarchical folder structure of .astrobee is copied to src. (The .git
  subfolder is skipped.)
- Plain files in .astrobee are symlinked to the identical locations in src, but
  with the following transformations applied:
  - Any file starting with a "ros1-" or "ros2-" prefix is (1) only symlinked if
    it the prefix matches the current ROS version, and (2) the symlinked
    filename has the prefix stripped.
  - Any file that starts with "jinja-" is interpreted as a Jinja2
    template, which is rendered and output to the same filename with the
    "jinja-" prefix stripped.  The Jinja2 rendering context contains the
    boolean variable USE_ROS1 that can be used in Jinja2 conditionals to
    modify the content between ROS versions.
This transformation has the following nice properties:
- It is performing an out-of-source build such that the ".astrobee" folder used
  with git is not polluted with auto-generated files or auto-selected symlinks.
- The "src" folder is not precious and can be automatically regenerated from
  ".astrobee" at any time.
- A single ".astrobee" git checkout folder can be used with both ROS1 and ROS2
  workspaces, so you can make a modification once and test it in both
  workspaces before you push it.
This script is fancier than the bare minimum because it tries to make the
minimum possible changes to "src" each time it is run, so as not to disturb the
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


def src_path(git_src_path):
    """
    Return the path under "src" corresponding to the specified path
    under "git_src".
    """
    return git_src_path.replace(".astrobee", "src", 1)


def rel_symlink(src, dst):
    """
    Return a relative symlink descriptor that if written to dst will
    point to src.
    """
    return {"type": "l", "target": os.path.relpath(src, os.path.dirname(dst))}


def jinja_render(path, use_ros1):
    """
    Return the result of rendering the Jinja2 template file found at
    path. Set the USE_ROS1 variable in the Jinja2 rendering context
    based on the value of use_ros1.
    """
    jinja_env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(os.path.dirname(path))
    )
    tmpl = jinja_env.get_template(os.path.basename(path))
    return tmpl.render({"USE_ROS1": use_ros1})


def get_current_state():
    """
    Return the current state of the "src" folder, expressed as a
    dictionary mapping paths under "src" to their current type and
    content.
    """
    result = {}
    for dir_path, dir_names, file_names in os.walk("src", followlinks=False):
        for d in dir_names:
            src_d = os.path.join(dir_path, d)
            result[src_d] = {"type": "d"}

        for f in file_names:
            src_f = os.path.join(dir_path, f)

            if os.path.islink(src_f):
                result[src_f] = {"type": "l", "target": os.readlink(src_f)}
            else:
                with open(src_f, "r") as stream:
                    result[src_f] = {"type": "f", "content": stream.read()}
    return result


def get_desired_state(use_ros1):
    """
    Return the desired state of the "src" folder, expressed as a
    dictionary mapping paths under "src" to their desired type and
    content.
    """
    result = {}
    for dir_path, dir_names, file_names in os.walk(".astrobee", followlinks=True):
        if dir_path == ".astrobee" and ".git" in dir_names:
            dir_names.remove(".git")  # skip .git subfolder

        for d in dir_names:
            src_d = os.path.join(src_path(dir_path), d)
            result[src_d] = {"type": "d"}

        for f in file_names:
            git_src_f = os.path.join(dir_path, f)
            src_f = src_path(git_src_f)

            if f.startswith("ros1-"):
                if use_ros1:
                    src_f = src_f.replace("ros1-", "", 1)
                    result[src_f] = rel_symlink(git_src_f, src_f)
                else:
                    pass  # don't symlink

            elif f.startswith("ros2-"):
                if use_ros1:
                    pass  # don't symlink
                else:
                    src_f = src_f.replace("ros2-", "", 1)
                    result[src_f] = rel_symlink(git_src_f, src_f)

            elif f.startswith("jinja-") or ".jinja." in f:
                src_f = src_f.replace("jinja-", "", 1)
                result[src_f] = {
                    "type": "f",
                    "content": jinja_render(git_src_f, use_ros1),
                }

            else:
                result[src_f] = rel_symlink(git_src_f, src_f)

    return result


def get_update_map(current_state, desired_state):
    """
    Generate an update map based on comparing the current and desired
    state. The update map is a dictionary mapping each path under "src"
    that requires an updates to a record containing that path's current
    and desired states, plus an description of the required update
    action.
    """
    src_paths = set(current_state.keys())
    src_paths.update(desired_state.keys())
    update_map = {
        p: {"current": current_state.get(p), "desired": desired_state.get(p)}
        for p in src_paths
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


def autogen_ros_version_src(use_ros1, dry_run):
    logging.warning("generating src directory from git_src...")
    current_state = get_current_state()
    desired_state = get_desired_state(use_ros1)
    update_map = get_update_map(current_state, desired_state)
    apply_update_map(update_map, dry_run)
    logging.warning("generating src directory from git_src... done")


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
        "-1",
        "--use-ros1",
        help="use ROS1 (ROS2 is used by default)",
        default=False,
        action="store_true",
    )
    parser.add_argument(
        "-d",
        "--dry-run",
        help="don't actually apply updates",
        default=False,
        action="store_true",
    )

    args = parser.parse_args()

    if not os.path.isdir(".astrobee"):
        parser.error("this script must be run from parent directory of git_src!")
    if os.path.exists("src/.git"):
        parser.error("src directory must not be a git checkout!")

    level = logging.WARNING - (10 * sum(args.verbose))
    logging.basicConfig(level=level, format="%(message)s")
    autogen_ros_version_src(args.use_ros1, args.dry_run)


if __name__ == "__main__":
    main()