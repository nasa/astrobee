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
Generate various plots that may help with debugging pico_utils issues.
"""

from __future__ import print_function

import matplotlib

matplotlib.use("Agg")

import argparse
import itertools
import logging

import numpy as np
import rosbag
from matplotlib import collections as mc
from matplotlib import pyplot as plt

import pico_utils as pico


def plot_xy_grid_many(inbag_path, verbose=False, fast=False, cam=pico.DEFAULT_CAM):
    """
    Verify that over multiple point cloud frames, the xy values for z =
    1 remain consistent.

    Different frames will be plotted in different colors; you should
    generally see only the last plotted color, except for a few
    scattered points where one frame or another is missing data. And all
    points should fall on a "warped grid" pattern based on the camera
    intrinsics.
    """
    print()
    print("=== plot_xy_grid_many ===")

    fig = plt.figure()
    fig, ax = plt.subplots()
    ax.axis("equal")

    for pts in pico.get_msgs(inbag_path, pico.get_pts_topic(cam), fast):
        xyz = pico.xyz_from_points(pts)
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        xp = x / z
        yp = y / z
        plt.plot(xp, yp, ".")

    plt.tight_layout()
    fig_path = "xy_grid_many.png"
    fig.set_size_inches((40, 40))
    plt.savefig(fig_path)
    logging.info("wrote to %s", fig_path)
    plt.close()


def iterate_pixels(img):
    """
    Given @img an N-channel image with dimensions (H, W, N), returns a
    generator that iterates through the pixels, where the value of each
    pixel is a length-N array.
    """
    w, h, _ = img.shape
    for ix in range(w):
        for iy in range(h):
            yield img[ix, iy, :]


def debug_plot_grid_lines(igrid):
    """
    Given @igrid a 2-channel "grid" image with dimensions (H, W, 2)
    where channels 0 and 1 are interpreted as x and y coordinates, plot
    the edges that connect Manhattan neighbor (x, y) points in order to
    visually verify that the grid is sensible.
    """
    lines = []

    # logging.debug("grid shapes %s %s", g.igrid[:, :-1, :].shape, g.igrid[:, 1:, :].shape)
    grid_pairs = (
        # horizontal neighbor pairs
        (igrid[:, :-1, :], igrid[:, 1:, :]),
        # vertical neighbor pairs
        (igrid[:-1, :, :], igrid[1:, :, :]),
    )
    for from_grid, to_grid in grid_pairs:
        for from_pt, to_pt in itertools.izip(
            iterate_pixels(from_grid), iterate_pixels(to_grid)
        ):
            # logging.debug("from_pt %s to_pt %s", from_pt, to_pt)
            if not (np.isnan(from_pt[0]) or np.isnan(to_pt[0])):
                lines.append((from_pt, to_pt))
                # logging.debug("lines d/d0 %f", np.linalg.norm(to_pt - from_pt) / 0.005)

    lc = mc.LineCollection(lines, linewidths=[0.1] * len(lines))
    fig, ax = plt.subplots()
    ax.add_collection(lc)
    ax.autoscale()


def debug_plot_grid_points(igrid):
    """
    Given @igrid a 2-channel "grid" image with dimensions (H, W, 2)
    where channels 0 and 1 are interpreted as x and y coordinates, plot
    the (x, y) points.
    """
    plt.plot(igrid[:, :, 0], igrid[:, :, 1], "r.", markersize=0.1)


def xy_from_xyz(xyz):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    xp = x / z
    yp = y / z
    xy = np.column_stack([xp, yp])
    return xy


def igrid_from_xyz(xyz):
    xy = xy_from_xyz(xyz)
    return xy.reshape((pico.SENSOR_HEIGHT_PIXELS, pico.SENSOR_WIDTH_PIXELS, 2))


def get_igrid(inbag_path, fast=False, cam=pico.DEFAULT_CAM):
    xyz = pico.merge_point_clouds(
        pico.get_msgs(inbag_path, pico.get_pts_topic(cam), fast=fast)
    )
    return igrid_from_xyz(xyz)


def plot_xy_grid_merge(inbag_path, verbose=False, fast=False, cam=pico.DEFAULT_CAM):
    """
    Verify the sanity of the x/y grid after merging all xyz frames.
    It should follow a "warped grid" pattern.
    """
    print()
    print("=== plot_xy_grid_merge ===")
    igrid = get_igrid(inbag_path, fast=fast, cam=cam)

    fig = plt.figure()
    fig, ax = plt.subplots()
    ax.axis("equal")

    debug_plot_grid_lines(igrid)
    debug_plot_grid_points(igrid)

    plt.tight_layout()
    fig_path = "xy_grid_merge.pdf"
    fig.set_size_inches((40, 40))
    plt.savefig(fig_path)
    logging.info("wrote to %s", fig_path)
    plt.close()


def plot_msg_sync(inbag_path, verbose=False, cam=pico.DEFAULT_CAM):
    """
    Figure out how to match extended messages to points messages from
    the same frame.

    From the resulting plots, it looks like generally when a new frame
    arrives from the sensor, its "points" message will be published
    first and its "extended" message will be published < 100 ms
    later. (Going by the msg.header.stamp publish timestamp.)
    """
    print()
    print("=== plot_msg_sync ===")

    with rosbag.Bag(inbag_path, "r") as inbag:
        topics = [pico.get_ext_topic(cam), pico.get_pts_topic(cam)]
        topic_times = {}
        for topic, msg, t in inbag.read_messages(topics):
            times = topic_times.setdefault(topic, [])
            times.append(msg.header.stamp)
            if len(times) >= 1000:
                break

    t0 = topic_times[pico.get_pts_topic(cam)][0].to_sec()
    for topic in topics:
        topic_times[topic] = np.array([t.to_sec() for t in topic_times[topic]]) - t0

    topic_data = {}
    WIN_RADIUS = 10
    DT_MIN = -2
    DT_MAX = 2

    pts_times = topic_times[pico.get_pts_topic(cam)]
    for topic in topics:
        data = []
        times = topic_times[topic]
        for i, pts_t in enumerate(pts_times):
            dt = times[(i - WIN_RADIUS) : (i + WIN_RADIUS)] - pts_t
            in_bounds = (dt >= DT_MIN) & (dt <= DT_MAX)
            dt = dt[in_bounds]
            for dt_val in dt:
                data.append((pts_t, dt_val))
        topic_data[topic] = np.array(data)

    fig = plt.figure()
    for topic in topics:
        data = topic_data[topic]
        plt.plot(data[:, 0], data[:, 1], ",", markersize=0.2)
    plt.legend(topics)
    plt.xlabel("elapsed time msg.header.stamp (s)")
    plt.ylabel("dt (s)")
    plt.yticks(np.arange(DT_MIN, DT_MAX + 0.1, 0.1))
    plt.grid()

    fig_path = "message_sync_all.pdf"
    fig.set_size_inches((50, 10))
    plt.savefig(fig_path)
    logging.info("wrote to %s", fig_path)
    plt.close()


def plot_first_distance_image(inbag_path, verbose=False, cam=pico.DEFAULT_CAM):
    """
    Save the distance image extracted from the first extended message in
    @inbag_path as an imshow() plot.
    """
    print()
    print("=== plot_first_distance_image ===")
    ext_msgs = pico.get_msgs(inbag_path, pico.get_ext_topic(cam))
    distance, amplitude, intensity, noise = pico.split_extended(next(ext_msgs))

    plt.figure()
    plt.imshow(distance)
    plt.tight_layout()
    fig_path = "distance_image.png"
    plt.savefig(fig_path)
    logging.info("wrote to %s", fig_path)
    plt.close()


def debug_xyz_agreement(inbag_path, verbose=False, fast=False, cam=pico.DEFAULT_CAM):
    print()
    print("=== debug_xyz_agreement ===")
    xyz_coeff = pico.get_xyz_coeff(inbag_path, fast)
    pair_stream = pico.get_ext_pts_pairs(inbag_path, fast=fast, cam=cam)

    err_max_log = []
    err_rms_log = []
    for i, (ext_msg, pts_msg) in enumerate(pair_stream):
        distance, amplitude, intensity, noise = pico.split_extended(ext_msg)
        distance = distance.reshape(
            (pico.SENSOR_HEIGHT_PIXELS * pico.SENSOR_WIDTH_PIXELS,)
        )
        xyz_ext = xyz_coeff * distance[:, np.newaxis]
        xyz_pts = pico.xyz_from_points(pts_msg)
        success, err_max, err_rms = pico.check_xyz_error(xyz_pts, xyz_ext, i)
        err_max_log.append(err_max)
        err_rms_log.append(err_rms)

    logging.info("Summary over all messages:")
    logging.info(" Error max (um): %.3f", np.max(err_max_log) * 1e6)
    logging.info(" Mean error RMS (um): %.3f", np.mean(err_rms_log) * 1e6)


if __name__ == "__main__":
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
    parser.add_argument("inbag", help="input bag")

    args = parser.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(message)s")

    plot_xy_grid_many(args.inbag, verbose=args.verbose, fast=args.fast, cam=args.cam)
    plot_xy_grid_merge(args.inbag, verbose=args.verbose, fast=args.fast, cam=args.cam)
    plot_msg_sync(args.inbag, verbose=args.verbose, cam=args.cam)
    plot_first_distance_image(args.inbag, verbose=args.verbose, cam=args.cam)
    debug_xyz_agreement(args.inbag, verbose=args.verbose, fast=args.fast, cam=args.cam)

    # suppress confusing ROS message at exit
    logging.getLogger().setLevel(logging.WARN)
