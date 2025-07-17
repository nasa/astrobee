#!/usr/bin/python3
import argparse
import csv
import sys

import rosbag
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion

parser = argparse.ArgumentParser(
    description="Takes bag created from ground truth script and extracts postion,orientaion, and time"
)

parser.add_argument(
    "--bag", "-b", type=str, help="bag created from ground truth script"
)

parser.add_argument("-file_name", type=str, help="name the file")

args = parser.parse_args()

input_bag = rosbag.Bag(args.bag)
mapping_pose = []
for topic, msg, t in input_bag.read_messages(topics="/sparse_mapping/pose"):
    time = msg.header.stamp
    for i, pose in enumerate(msg.poses):
        point_x = pose.postion.x
        point_y = pose.postion.y
        point_z = pose.postion.z
        ori_x = pose.orientation.x
        ori_y = pose.orientation.y
        ori_z = pose.orientation.z
        ori_w = pose.orientation.w
        info = (time, point_x, point_y, point_z, ori_x, ori_y, ori_z, ori_w)
        mapping_pose.append(info)

if args.file_name is not None:
    poses_file = "pose_" + args.file_name + ".csv"
else:
    split = input_bag.split(".", 1)
    file_name = split[0]
    poses_file = file_name + ".csv"

with open(poses_file, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["time", "posex", "posey", "posez", "orix", "oriy", "oriz", "oriw"])
    for tup in mapping_pose:
        writer.writerow(tup)
