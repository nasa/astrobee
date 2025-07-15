#!/usr/bin/python3
import rosbag
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import csv
import sys
import argparse

parser = argparse.ArgumentParser(description="Takes bag created from ground truth script and extracts postion,orientaion, and time")

parser.add_argument(
    "-bag",
    "-b",
    type=str,
    help="bag created from ground truth script"
)

parser.add_argument(
    "--file_name",
    "-f",
    type=str,
    help="name the file"
)

args = parser.parse_args()

input_bag = rosbag.Bag(args.bag)
mapping_pose = []
for topic, msg, t in input_bag.read_messages(topics="/sparse_mapping/pose"):
    time = msg.header.time
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
    poses_file = "pose_" + ".csv"
 
with open(poses_file, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["time", "posex", "posey", "posez", "orix", "oriy", "oriz", "oriw"])
    for tup in mapping_pose:
        writer.writerow(tup)
 
