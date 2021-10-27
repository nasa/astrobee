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

import argparse
import os
import sys

import rosbag


def get_bags_with_topic(bag_names, topic):
  bag_names_with_topic = []
  for bag_name in bag_names:
    with rosbag.Bag(bag_name, "r") as bag:
      if bag.get_message_count(topic) > 0:
        bag_names_with_topic.append(bag_name)
  return bag_names_with_topic


#TODO(rsoussan): combine this with same fcn in graph_bag/merge_bags.py
def find_bags_in_directory(directory):
  # Find bagfiles with bag prefix in current directory, fail if none found
  bag_names = [
    os.path.join(directory, bag)
    for bag in os.listdir(directory)
    if os.path.isfile(os.path.join(directory, bag)) and bag.endswith(".bag")
  ]
  if len(bag_names) == 0:
    print("No bag files found")
    sys.exit()
  else:
    print(("Found " + str(len(bag_names)) + " bag files."))
    for bag_name in bag_names:
      print(bag_name)
    return bag_names


def find_bags_with_topic(directory, topic):
  bag_names = find_bags_in_directory(directory)
  return get_bags_with_topic(bag_names, topic)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--cam-topic", default="/hw/cam_nav")
  parser.add_argument("-d", "--directory", default="./")
  args = parser.parse_args()
  bag_names_with_topic = find_bags_with_topic(args.directory, args.cam_topic)
  if len(bag_names_with_topic) == 0:
    print("No bag files with topic " + args.cam_topic + " found.")
    sys.exit()
  else:
    print(("Found " + str(len(bag_names_with_topic)) + " bag files with " + args.cam_topic + " topic."))
    for bag_name in bag_names_with_topic:
      print(bag_name)
