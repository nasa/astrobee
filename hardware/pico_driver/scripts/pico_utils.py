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
Utilities for working with Pico Flexx data in Python.
"""

from __future__ import print_function

import collections
import io
import itertools
import logging
import struct

import cv2
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField

PTS_TOPIC_TMPL = "/hw/depth_{cam}/points"
EXT_TOPIC_TMPL = "/hw/depth_{cam}/extended"
AMP_TOPIC_TMPL = "/hw/depth_{cam}/extended/amplitude_int"
CAM_CHOICES = ("haz", "perch")
DEFAULT_CAM = "haz"
SENSOR_WIDTH_PIXELS = 224
SENSOR_HEIGHT_PIXELS = 171

# We've observed that in the raw bags from Astrobee, the depth "extended"
# message is reliably published < 100 ms after the "points" message, when
# comparing the msg.header.stamp values. We can use this to robustly identify
# matched message pairs, even if they are occasionally out of order in the
# rosbag.
EXT_MINUS_PTS_DT_LIMIT = (0, 0.1)
PTS_MINUS_EXT_DT_LIMIT = (-EXT_MINUS_PTS_DT_LIMIT[1], -EXT_MINUS_PTS_DT_LIMIT[0])

# This tolerance is much stricter than what we really need from the application
# side, and about double the maximum error we've seen so far with
# reconstructing points messages from extended messages. If we start to see
# tolerance violations at this level, we could consider increasing it.
ERROR_TOLERANCE_METERS = 250e-6

# The amplitude portion of the extended message is encoded as a floating point
# value that needs to be scaled before converting to an integer type for Kalibr
# compatibility. The scaling factor is somewhat arbitrary, and Oleg verified
# this value works empirically, although it may not be optimal.
AMPLITUDE_SCALE = 100


def get_ext_topic(cam):
    return EXT_TOPIC_TMPL.format(cam=cam)


def get_pts_topic(cam):
    return PTS_TOPIC_TMPL.format(cam=cam)


def get_amp_topic(cam):
    return AMP_TOPIC_TMPL.format(cam=cam)


def rms(v):
    return np.sqrt(np.mean(v * v))


class PrintEveryK:
    """
    Acts a bit like a logger, but prints the message only one out of
    every k times you call it. Convenient to use with large k for
    occasionally indicating progress.
    """

    def __init__(self, k):
        self.k = k
        self.count = 0

    def debug(self, *args, **kwargs):
        self.count += 1
        if (self.count % self.k) == 0:
            logging.debug(*args, **kwargs)

    def info(self, *args, **kwargs):
        self.count += 1
        if (self.count % self.k) == 0:
            logging.info(*args, **kwargs)


def fast_filter(msgs):
    """
    Grabs an arbitrary subset of 100 messages from the stream.
    Convenient for fast testing.
    """
    return itertools.islice(msgs, 100, 1100, 10)


def get_msg_tuples_internal(inbag_path, topic):
    with rosbag.Bag(inbag_path, "r") as inbag:
        for msg_tuple in inbag.read_messages([topic]):
            yield msg_tuple


def get_msg_tuples(inbag_path, topic, fast=False):
    """
    A generator that iterates through message tuples (topic, msg, t)
    in the ROS bag @inbag_path on topic @topic. If @fast is True,
    only returns a subset of the bag for fast testing.
    """
    msg_tuples = get_msg_tuples_internal(inbag_path, topic)
    if fast:
        return fast_filter(msg_tuples)
    return msg_tuples


def get_msgs(inbag_path, topic, fast=False):
    """
    A generator that iterates through messages in the ROS bag
    @inbag_path on topic @topic. If @fast is True, only returns a subset
    of the bag for fast testing.
    """
    return (msg_tuple[1] for msg_tuple in get_msg_tuples(inbag_path, topic, fast))


def dt64(t):
    """
    Convert rospy.Time to np.datetime64.
    """
    return np.datetime64(t.to_nsec(), "ns")


def peek(iterable):
    """
    Python generators don't provide a peek() method to examine the first
    item in the sequence without actually popping it off. This function
    provides similar functionality if you call it like this:

    first, it = peek(it)

    After the call, @first will still be the first item in @it.
    """
    try:
        first = next(iterable)
    except StopIteration:
        return None, None
    return first, itertools.chain([first], iterable)


def split_msg_stream_by_topic(in_stream, topics):
    """
    Given @in_stream a message tuple generator and @topics a sequence of
    topics [topic_1, .., topic_n], returns a sequence of message tuple
    generators [stream_1, .., stream_n] such that stream_i generates
    the message tuples from @in_stream that match topic_i.
    """
    it = iter(in_stream)
    deques = [collections.deque() for i in range(len(topics))]
    deque_by_topic = dict(zip(topics, deques))

    def gen(deq):
        while 1:
            while not deq:
                try:
                    newval = next(it)
                except StopIteration:
                    return
                msg_topic, msg, t = newval
                # logging.debug("got message topic %s", msg_topic)
                deque_by_topic[msg_topic].append(newval)
            # logging.debug("propagated message topic %s", topic)
            yield deq.popleft()

    return tuple(gen(deq) for deq in deques)


def get_matched_msg_pairs(stream1, stream2, dt_limit=None):
    """
    Given @stream1 and @stream2 message tuple generators, returns
    matched pairs of messages from the two streams. The behavior is very
    similar to itertools.izip() followed by selecting just the message
    part of the (topic, message, t) tuple.

    However, if @dt_limit is specified, instead of simply matching
    message pairs that have the same index in their respective
    sequences, it also calculates their publish timestamp differential:

    dt = msg2.header.stamp - msg1.header.stamp

    Two messages are potential matches if dt falls within the specified
    interval @dt_limit = (dt_min, dt_max). Subject to that constraint,
    the pairing is greedy, and each message can be used at most once.
    If a particular message has no valid match based on dt, it will be
    dropped with a debug message.

    Using @dt_limit can make the matching more robust to occasional out
    of order rosbag logging.
    """
    while 1:
        val1, stream1 = peek(stream1)
        val2, stream2 = peek(stream2)
        if val1 is None or val2 is None:
            logging.debug(
                "get_matched_msg_pairs: ran out of pairs, stream1 %s, stream2 %s",
                "done" if val1 is None else "not done",
                "done" if val2 is None else "not done",
            )
            return
        topic1, msg1, t1 = val1
        topic2, msg2, t2 = val2
        if dt_limit is not None:
            dt_min, dt_max = dt_limit
            dt = (msg2.header.stamp - msg1.header.stamp).to_sec()
            if not (dt_min <= dt):
                # need a newer t2 to make dt large enough: drop t2
                logging.debug(
                    "get_matched_msg_pairs: dropping unmatched message from stream1, dt = %.3f, at %s",
                    dt,
                    dt64(t1),
                )
                next(stream2)
                continue
            if not (dt <= dt_max):
                # need a newer t1 to make dt small enough: drop t1
                logging.debug(
                    "get_matched_msg_pairs: dropping unmatched message from stream2, dt = %.3f, at %s",
                    dt,
                    dt64(t2),
                )
                next(stream1)
                continue
        yield msg1, msg2
        next(stream1)
        next(stream2)


def get_ext_pts_pairs_internal(inbag_path, cam):
    topics = [get_ext_topic(cam), get_pts_topic(cam)]
    with rosbag.Bag(inbag_path, "r") as inbag:
        msg_stream = inbag.read_messages(topics)
        ext_stream, pts_stream = split_msg_stream_by_topic(msg_stream, topics)
        # note: we can't just return the generator here because returning from this
        # function would close the bag and render the generator invalid.
        for pair in get_matched_msg_pairs(
            ext_stream, pts_stream, dt_limit=PTS_MINUS_EXT_DT_LIMIT
        ):
            yield pair


def get_ext_pts_pairs(inbag_path, fast=False, cam=DEFAULT_CAM):
    """
    Given @inbag_path a path to an input bag, returns a generator that
    will produce matched message pairs (ext_msg, pts_msg) from the bag
    on the "extended" and "points" topics, respectively.

    The message matching attempts to guarantee the matched messages are
    from the same sensor frame by comparing their publish timestamps,
    which is empirically more robust than relying on the sequence
    ordering in the bag.
    """
    pairs = get_ext_pts_pairs_internal(inbag_path, cam=cam)
    if fast:
        return fast_filter(pairs)
    return pairs


def xyz_from_points(pts):
    """
    Extract xyz points from a "points" message. Replaces zero entries
    with np.nan to explicitly mark them as invalid.
    """
    pc = pc2.read_points(pts, field_names=("x", "y", "z"))
    xyz = np.array(list(pc))

    # mark rows that have z == 0 as nan
    xyz[xyz[:, 2] == 0, :] = np.nan

    return xyz


def merge_point_clouds(pts_stream):
    """
    Given @pts_stream a stream of points messages, merges the xyz points
    from all messages into a single xyz frame.

    All points messages have the same length of xyz array, and each
    index within the array consistently corresponds to the same pixel of
    the sensor.

    The merge semantics produces an output xyz array where the value at
    each pixel is the mean value over all of the messages that have a
    valid value for that pixel (not nan).
    """
    xyz_sum = np.zeros(
        (SENSOR_HEIGHT_PIXELS * SENSOR_WIDTH_PIXELS, 3), dtype=np.float64
    )
    xyz_count = np.zeros((SENSOR_HEIGHT_PIXELS * SENSOR_WIDTH_PIXELS,), dtype=np.uint32)
    for i, pts in enumerate(pts_stream):
        xyz = xyz_from_points(pts)
        valid = ~np.isnan(xyz[:, 0])
        xyz_sum[valid] += xyz[valid]
        xyz_count[valid] += 1

    merged = np.empty((SENSOR_HEIGHT_PIXELS * SENSOR_WIDTH_PIXELS, 3), dtype=np.float64)
    merged.fill(np.nan)
    valid = xyz_count != 0
    merged[valid] = xyz_sum[valid] / xyz_count[valid, np.newaxis]

    logging.info("merge_point_clouds: processed %s point clouds", i + 1)
    num_invalid_pixels = np.count_nonzero(xyz_count == 0)
    logging.info(
        "merge_point_clouds: invalid pixels: %d (%.1f%%)",
        num_invalid_pixels,
        float(num_invalid_pixels) / (SENSOR_HEIGHT_PIXELS * SENSOR_WIDTH_PIXELS) * 100,
    )
    return merged


def split_extended(ext_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ext_msg.raw, desired_encoding="32FC4")
    distance, amplitude, intensity, noise = cv2.split(cv_image)

    # explicitly mark no-data value of 0 as nan
    distance[distance == 0] = [np.nan]

    return distance, amplitude, intensity, noise


def get_xyz_coeff(inbag_path, fast=False, cam=DEFAULT_CAM):
    xyz = merge_point_clouds(get_msgs(inbag_path, get_pts_topic(cam), fast=fast))
    xyz = xyz / np.linalg.norm(xyz, axis=1)[:, np.newaxis]
    return xyz


def xyz_from_distance(distance, xyz_coeff):
    return (
        xyz_coeff
        * distance.reshape((SENSOR_HEIGHT_PIXELS * SENSOR_WIDTH_PIXELS,))[:, np.newaxis]
    )


def xyz_from_extended(ext_msg, xyz_coeff):
    distance, amplitude, intensity, noise = split_extended(ext_msg)
    return xyz_from_distance(distance, xyz_coeff)


def make_amp_msg(header, amplitude):
    msg = Image()
    msg.header = header
    msg.height = SENSOR_HEIGHT_PIXELS
    msg.width = SENSOR_WIDTH_PIXELS
    msg.encoding = "mono16"
    msg.is_bigendian = False
    msg.step = SENSOR_WIDTH_PIXELS * 2

    # simulate OpenCV convertTo() operation used in pico_proxy
    max16 = (1 << 16) - 1
    amp16 = np.clip(np.rint(amplitude * AMPLITUDE_SCALE), 0, max16).astype(np.uint16)

    msg.data = amp16.tobytes()

    return msg


def make_points_msg(header, xyz):
    pts = PointCloud2()
    pts.header = header
    pts.height = SENSOR_HEIGHT_PIXELS
    pts.width = SENSOR_WIDTH_PIXELS
    pts.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    pts.is_bigendian = False
    pts.point_step = 20
    pts.row_step = 4480

    xyz32 = xyz.astype(np.float32)
    xyz32[np.isnan(xyz32)] = 0.0
    buf = io.BytesIO()
    for row in xyz32:
        # write xyz values (12 bytes)
        buf.write(struct.pack("<fff", *row))
        # pad with 8 bytes of 0 to replicate original point_step of 20
        buf.write(struct.pack("<q", 0))
    pts.data = buf.getvalue()

    pts.is_dense = True

    return pts


def check_equal(a, b, i, field_name):
    if a == b:
        return True
    else:
        if max(len(str(a)), len(str(b))) > 100:
            logging.info("FAILED at index %d: %s values differ", i, field_name)
        else:
            logging.info("FAILED at index %d: %s %s != %s", i, field_name, a, b)
        return False


def check_xyz_error(xyz1, xyz2, i):
    success = True

    # the same entries should be valid in both point clouds
    valid_diff = np.isnan(xyz1) ^ np.isnan(xyz2)
    if np.count_nonzero(valid_diff):
        logging.info("FAILED at index %d: messages have different valid pixels", i)
        success = False

    # check accuracy in valid areas
    valid = ~np.isnan(xyz1)
    err = xyz2[valid] - xyz1[valid]
    err_max = np.max(np.abs(err))
    err_rms = rms(err)

    logging.debug("%5d Error max (um): %.3f", i, err_max * 1e6)
    logging.debug("%5d Error RMS (um): %.3f", i, err_rms * 1e6)
    if err_max > ERROR_TOLERANCE_METERS:
        logging.info("FAILED at index %d: Error max (um) = %.3f", i, err_max * 1e6)
        success = False

    return success, err_max, err_rms


def check_points_msg_pair(orig_msg, test_msg, i, verbose=False):
    success = True

    success &= check_equal(orig_msg.height, test_msg.height, i, "height")
    success &= check_equal(orig_msg.width, test_msg.width, i, "width")
    success &= check_equal(orig_msg.fields, test_msg.fields, i, "fields")
    success &= check_equal(
        orig_msg.is_bigendian, test_msg.is_bigendian, i, "is_bigendian"
    )
    success &= check_equal(orig_msg.point_step, test_msg.point_step, i, "point_step")
    success &= check_equal(orig_msg.row_step, test_msg.row_step, i, "row_step")
    success &= check_equal(orig_msg.is_dense, test_msg.is_dense, i, "is_dense")

    orig_xyz = xyz_from_points(orig_msg)
    test_xyz = xyz_from_points(test_msg)

    xyz_success, err_max, err_rms = check_xyz_error(orig_xyz, test_xyz, i)
    success &= xyz_success

    return success, err_max, err_rms


def check_amp_msg_pair(orig_msg, test_msg, i, verbose=False):
    success = True

    success &= check_equal(orig_msg.height, test_msg.height, i, "height")
    success &= check_equal(orig_msg.width, test_msg.width, i, "width")
    success &= check_equal(
        orig_msg.is_bigendian, test_msg.is_bigendian, i, "is_bigendian"
    )
    success &= check_equal(orig_msg.step, test_msg.step, i, "step")
    success &= check_equal(orig_msg.data, test_msg.data, i, "data")

    return success
