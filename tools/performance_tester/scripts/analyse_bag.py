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

import rosbag

cpu_topic = "/mgt/cpu_monitor/state"

class Topic:
    def __init__(self, name):
        self.name = name
        self.hz_max = 0
        self.hz_min = 100
        self.hz = -1
        self.t_last = -1

    def addTimestamp(self, t):
        # Check if it is the fist available measurement
        if (self.t_last = -1):
            self.t_last = t
            return

        # Calculate parameters
        if (t == self.t_last):
            print("same pub time")
            return
        self.hz = 1 / (t - self.t_last)
        if (self.hz > self.hz_max):
            self.hz_max = self.hz
        if (self.hz < self.hz_min):
            self.hz_min = self.hz

        # Update parameters
        self.t_last = t

    def printResult(self):
        # Plot the frequency over time

class CpuUsage:
    def __init__(self):
        self.nodes = []

    def addCpuMessage(self, msg):
        # Read message parameters


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-name", default="")
    parser.add_argument('-n', '--topic-list', nargs='+', default=[])

    args = parser.parse_args()

    # Create dictionary
    freq_analyzer = dict ((topic, Topic(topic)) for topic in args.topic_list)
    cpu_analyzer = CpuUsage()

    # Go through the messages in the bag file
    for topic_bag, msg, t in rosbag.Bag(args.bag_name).read_messages():
        if topic == cpu_topic:
            cpu_analyzer.addCpuMessage(msg)
            continue
        # Check if the topic matches
        for topic in args.topic_list:
            if topic == topic_bag:
                # Calculate hz
                freq_analyzer[topic].addTimestamp(t.to_sec())
