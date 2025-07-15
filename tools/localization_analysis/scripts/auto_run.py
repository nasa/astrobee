#!/usr/bin/python3
import rosbag
import subprocess
import sys
import argparse

parser = argparse.ArgumentParser(description="This file is intended to run the ground truth scripts on mutiple bags in the same directory with one map")

#define arguments
parser.add_argument(
    "-bag_path",
    "-bp",
    type=str,
    help="path of the directory containing your bags"
)

parser.add_argument(
    "-map_path",
    "-mp",
    type=str,
    help="path of the directory containing your map"
)

parser.add_argument(
    "-bags",
    "-b",
    nargs='+',
    type=str,
    help="List of bag names to process"
)

args = parser.parse_args()
# takes arugment and sets bag_file_path to the path of the directory containing your bags ex: path/to/bag/
bag_file_path = args.bag_path

# takes arugment and sets map_file_pat to the path of the directory containing your map
map_file_path = args.map_path

# stores the names of the bag files you want to run the ground truth script on
bags = args.bags

for bag in bags:
    subprocess.run(["rosrun", "localization_analysis", "make_groundtruth.py", bag_file_path + bag, map_file_path])
