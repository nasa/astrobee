#!/usr/bin/python3
import rosbag
import subprocess
import sys
bag_file_path = "your file path to bags"
map_file_path = "file path to maps"
bags = []

for bag in bags:
    subprocess.run(["rosrun", "localization_analysis", "make_groundtruth.py", bag_file_path + bag, map_file_path])
