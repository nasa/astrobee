#!/usr/bin/python3

import argparse
import os
import sys

import matplotlib

import loc_states
import plot_helpers
import poses
import rmse_utilities
import utilities
import vector3d_plotter
import velocities

matplotlib.use("pdf")
import csv
import math

import geometry_msgs
import matplotlib.pyplot as plt
import rosbag
from plot_results import load_pose_msgs
from matplotlib.backends.backend_pdf import PdfPages

def add_plots(
    pdf,
    surf_poses,
    brisk_poses,
    groundtruth_poses,
):
    # colors = ["r", "b", "g"]
    position_plotter = vector3d_plotter.Vector3dPlotter(
        "Time (s)", "Position (m)", "Groundtruth vs. Sparse Mapping Position", True
    )
    if brisk_poses:
        position_plotter.add_pose_position(
            brisk_poses,
            linestyle="None",
            colors = ["b", "b", "b"],
            marker="o",
            markeredgewidth=0.1,
            markersize=1.5,
            name="BRISK"
        )
    position_plotter.add_pose_position(
        surf_poses,
        linestyle="None",
        colors = ["r", "r", "r"],
        marker="o",
        markeredgewidth=0.1,
        markersize=1.5,
        name="SURF"
    )

    if groundtruth_poses:
        position_plotter.add_pose_position(
            groundtruth_poses,
            linestyle="None",
            colors = ["g", "g", "g"],
            marker="o",
            markeredgewidth=0.1,
            markersize=1.5,
            name="Groundtruth"
        )
    position_plotter.plot(pdf)

    # orientations
    orientation_plotter = vector3d_plotter.Vector3dPlotter(
        "Time (s)", "Orientation (deg)", "Groundtruth vs. Sparse Mapping Orientation", True
    )
    if brisk_poses:
        orientation_plotter.add_pose_orientation(
            brisk_poses,
            linestyle="None",
            marker="o",
            colors = ["b", "b", "b"],
            markeredgewidth=0.1,
            markersize=1.5,
            name="BRISK"
        )
    orientation_plotter.add_pose_orientation(
        surf_poses,
        linestyle="None",
        marker="o",
        colors = ["r", "r", "r"],
        markeredgewidth=0.1,
        markersize=1.5,
        name="SURF"
    )
    if groundtruth_poses:
        orientation_plotter.add_pose_orientation(
            groundtruth_poses,
            linestyle="None",
            marker="o",
            colors = ["g", "g", "g"],
            markeredgewidth=0.1,
            markersize=1.5,
            name="Groundtruth"
        )
    orientation_plotter.plot(pdf)

# Groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def create_plots(
    surf_bagfile,
    brisk_bagfile,
    groundtruth_bagfile,
    output_pdf_file,
):
    surf_bag = rosbag.Bag(surf_bagfile)
    brisk_bag = rosbag.Bag(brisk_bagfile) if brisk_bagfile else None
    groundtruth_bag = rosbag.Bag(groundtruth_bagfile) if groundtruth_bagfile else None
    bag_start_time = surf_bag.get_start_time()

    surf_poses = poses.Poses("Sparse Mapping", "/sparse_mapping/pose")
    brisk_poses = poses.Poses("Sparse Mapping", "/sparse_mapping/pose") if brisk_bag else None
    groundtruth_poses = poses.Poses("Sparse Mapping", "/sparse_mapping/pose")  if groundtruth_bagfile else None
    surf_vec_of_poses = [surf_poses]
    brisk_vec_of_poses = [brisk_poses] if brisk_bag else None
    groundtruth_vec_of_poses = [groundtruth_poses]  if groundtruth_bagfile else None
    load_pose_msgs(surf_vec_of_poses, surf_bag, bag_start_time)
    load_pose_msgs(brisk_vec_of_poses, brisk_bag, bag_start_time) if brisk_bag else None
    load_pose_msgs(groundtruth_vec_of_poses, groundtruth_bag, bag_start_time)  if groundtruth_bagfile else None

    surf_bag.close()
    if brisk_bag is not None:
        brisk_bag.close()
    if groundtruth_bag is not None:
        groundtruth_bag.close()

    with PdfPages(output_pdf_file) as pdf:
        add_plots(
            pdf,
            surf_poses,
            brisk_poses,
            groundtruth_poses,
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-s", "--surf-bagfile", help="Input surf bagfile.")
    parser.add_argument(
        "-b",
        "--brisk-bagfile",
        default=None,
        help="Input brisk bagfile.",
    )
    parser.add_argument(
        "-g",
        "--groundtruth-bagfile",
        default=None,
        help="bagfile containing groundtruth poses to use as a comparison for poses in the input bagfile. If none provided, sparse mapping poses are used as groundtruth from the input bagfile if available.",
    )
    parser.add_argument("--output-file", default="output.pdf", help="Output pdf file.")
    args = parser.parse_args()
    if not os.path.isfile(args.surf_bagfile):
        print(("Bag file " + args.surf_bagfile + " does not exist."))
        sys.exit()
    if args.brisk_bagfile is not None and not os.path.isfile(args.brisk_bagfile):
        print(("Bag file " + args.brisk_bagfile + " does not exist."))
        sys.exit()
    if args.groundtruth_bagfile and not os.path.isfile(args.groundtruth_bagfile):
        print(("Groundtruth Bag file " + args.groundtruth_bagfile + " does not exist."))
        sys.exit()
    create_plots(
        args.surf_bagfile,
        args.brisk_bagfile,
        args.groundtruth_bagfile,
        args.output_file
    )
