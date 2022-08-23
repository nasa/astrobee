#!/usr/bin/env python

"""
Calculate statistics on incoming images, either received live or read from
a bag.
"""

import argparse
import signal
import sys

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

DEFAULT_IMAGE_TOPIC = "/hw/cam_nav"
STRIDE = 50
THROTTLE_PERIOD_MS = 1000

throttle_last_image_time_g = None
bridge_g = CvBridge()


def sigint_handler(signum, frame):
    rospy.signal_shutdown("Caught SIGINT such as user Ctrl-C, exiting")


def get_image_mean(img):
    ny, nx = img.shape
    tot = 0
    count = 0
    for y in range(0, ny, STRIDE):
        for x in range(0, nx, STRIDE):
            tot += img[y, x]
            count += 1
    return (float(tot) * 5) / (count * 255)


def img_handler(imgmsg):
    global throttle_last_image_time_g
    global bridge_g

    # implement throttling
    t = imgmsg.header.stamp.to_sec()
    if (
        throttle_last_image_time_g
        and (t - throttle_last_image_time_g) * 1000 < THROTTLE_PERIOD_MS
    ):
        return
    throttle_last_image_time_g = t

    # actual handler
    img = bridge_g.imgmsg_to_cv2(imgmsg)
    mean = get_image_mean(img)
    print("Time: %14.1f   Mean: %5.1f" % (t, mean))


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter):
    pass


def image_stats(topic=None, bag_path=None):
    if bag_path is None:
        # subscribe for live messages
        sub = rospy.Subscriber(topic, Image, img_handler)
        rospy.spin()
    else:
        # read messages from bag for testing
        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages([topic]):
                img_handler(msg)


def main():
    signal.signal(signal.SIGINT, sigint_handler)

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-b", "--bag", help="read images from the specified bag instead of subscribing"
    )
    parser.add_argument(
        "-t",
        "--topic",
        help="process images on the specified topic",
        default=DEFAULT_IMAGE_TOPIC,
    )
    args = parser.parse_args()
    rospy.init_node("image_stats")

    image_stats(topic=args.topic, bag_path=args.bag)


if __name__ == "__main__":
    main()
