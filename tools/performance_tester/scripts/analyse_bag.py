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

import matplotlib.pyplot as plt
import numpy as np
import rosbag

EPS = 1e-8
cpu_topic = "/mgt/cpu_monitor/state"


class Topic:
    def __init__(self, name, n, n_total):
        self.n_topic_total = n_total
        self.n_topic = n
        self.name = name
        self.hz_max = 0
        self.hz_min = 100
        self.time = []
        self.frequency = []
        self.t_diff = []
        self.t_last = -1

    def addTimestamp(self, t):
        # Check if it is the fist available measurement
        if self.t_last == -1:
            self.t_last = t
            return

        # Check if we have repeated timestamps
        if t - self.t_last < EPS:
            print("same pub time")
            return

        hz = 1 / (t - self.t_last)
        self.frequency.append(hz)
        self.t_diff.append(t - self.t_last)
        self.time.append(t)

        if hz > self.hz_max:
            self.hz_max = hz
        if hz < self.hz_min:
            self.hz_min = hz

        # Update parameters
        self.t_last = t

    def printResult(self):
        if self.n_topic == 0:
            plt.figure(100, figsize=(16, 16))

        # Plot the frequency over time
        xpoints = np.array(self.time) - self.time[0]
        ypoints = np.array(self.frequency)

        plt.subplot(self.n_topic_total, 2, 2 * self.n_topic + 1)
        plt.plot(xpoints, ypoints)
        plt.xlabel("Time(s)")
        plt.ylabel("Frequency (Hz)")
        plt.title(self.name)

        plt.subplot(self.n_topic_total, 2, 2 * self.n_topic + 2)
        num_bins = 100
        n, bins, patches = plt.hist(self.t_diff, num_bins, facecolor="blue", alpha=0.5)


class CpuUsage:
    def __init__(self):
        self.nodes = []
        self.mlp_cpu_loads = [[], [], [], []]
        self.mlp_node_loads = []
        self.mlp_time = []
        self.llp_cpu_loads = [[], []]
        self.llp_load_loads = []
        self.llp_time = []

        self.llp_nodes = {}
        self.llp_nodes_time = []
        self.mlp_nodes = {}
        self.mlp_nodes_time = []

    def addCpuMessage(self, msg):
        # We want the total cpu usage
        total_cpu = 0
        for field in msg.load_fields:
            if field == msg.TOTAL:
                break
            total_cpu = total_cpu + 1

        if msg.name == "llp":
            self.llp_time.append(msg.header.stamp.to_sec())
            for i in range(len(msg.cpus)):
                self.llp_cpu_loads[i].append(msg.cpus[i].loads[total_cpu])

        if msg.name == "mlp":
            self.mlp_time.append(msg.header.stamp.to_sec())
            for i in range(len(msg.cpus)):
                self.mlp_cpu_loads[i].append(msg.cpus[i].loads[total_cpu])

        # Save nodes info
        append_time_mlp = False
        append_time_llp = False
        for node in msg.load_nodes:
            if "mlp" in node.name:
                append_time_mlp = True
                if node.name in self.mlp_nodes:
                    self.mlp_nodes[node.name].append(node.load)
                else:
                    self.mlp_nodes[node.name] = [node.load]
            if "llp" in node.name:
                append_time_llp = True
                if node.name in self.llp_nodes:
                    self.llp_nodes[node.name].append(node.load)
                else:
                    self.llp_nodes[node.name] = [node.load]
        if append_time_mlp:
            self.mlp_nodes_time.append(msg.header.stamp.to_sec())
        if append_time_llp:
            self.llp_nodes_time.append(msg.header.stamp.to_sec())

    def printResult(self):
        fig, ax = plt.subplots(3, 2, figsize=(16, 16))

        # Plot LLP CPU usage
        xpoints = np.array(self.llp_time) - self.llp_time[0]
        for cpu in range(len(self.llp_cpu_loads)):
            ypoints = np.array(self.llp_cpu_loads[cpu])
            ax[0, 0].plot(xpoints, ypoints, label="cpu" + str(cpu))
        ax[0, 0].set_xlabel("Time(s)")
        ax[0, 0].set_ylabel("Load")
        ax[0, 0].set_title("LLP CPU loads")
        ax[0, 0].legend()

        # Plot MLP CPU usage
        xpoints = np.array(self.mlp_time) - self.llp_time[0]
        for cpu in range(len(self.mlp_cpu_loads)):
            ypoints = np.array(self.mlp_cpu_loads[cpu])
            ax[0, 1].plot(xpoints, ypoints, label="cpu" + str(cpu))
        ax[0, 1].set_xlabel("Time(s)")
        ax[0, 1].set_ylabel("Load")
        ax[0, 1].set_title("MLP CPU loads")
        ax[0, 1].legend()

        # Plot nodes usage
        xpoints = np.array(self.llp_nodes_time) - self.llp_nodes_time[0]
        for llp_node in self.llp_nodes:
            ax[1, 0].plot(xpoints, self.llp_nodes[llp_node], label=llp_node, alpha=0.8)
        ax[1, 0].set_xlabel("Time(s)")
        ax[1, 0].set_ylabel("Load")
        ax[1, 0].set_title("Node CPU loads")
        ax[1, 0].legend()

        xpoints = np.array(self.mlp_nodes_time) - self.mlp_nodes_time[0]
        for mlp_node in self.mlp_nodes:
            ax[1, 1].plot(xpoints, self.mlp_nodes[mlp_node], label=mlp_node, alpha=0.8)
        ax[1, 1].set_xlabel("Time(s)")
        ax[1, 1].set_ylabel("Load")
        ax[1, 1].set_title("Node CPU loads")
        ax[1, 1].legend()

        # Plot nodes usage cummulative
        ax[2, 0].stackplot(
            np.array(self.llp_nodes_time) - self.llp_nodes_time[0],
            self.llp_nodes.values(),
            labels=self.llp_nodes.keys(),
            alpha=0.8,
        )
        ax[2, 0].set_xlabel("Time(s)")
        ax[2, 0].set_ylabel("Load")
        ax[2, 0].set_title("Node CPU loads")
        ax[2, 0].legend()

        ax[2, 1].stackplot(
            np.array(self.mlp_nodes_time) - self.mlp_nodes_time[0],
            self.mlp_nodes.values(),
            labels=self.mlp_nodes.keys(),
            alpha=0.8,
        )
        ax[2, 1].set_xlabel("Time(s)")
        ax[2, 1].set_ylabel("Load")
        ax[2, 1].set_title("Node CPU loads")
        ax[2, 1].legend()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-name", default="")
    parser.add_argument("-n", "--topic-list", nargs="+", default=[])

    args = parser.parse_args()

    # Create dictionary
    freq_analyzer = dict(
        (topic, Topic(topic, i, len(args.topic_list)))
        for i, topic in enumerate(args.topic_list)
    )
    cpu_analyzer = CpuUsage()

    # Go through the messages in the bag file
    for topic_bag, msg, t in rosbag.Bag(args.bag_name).read_messages():
        if topic_bag == cpu_topic:
            cpu_analyzer.addCpuMessage(msg)
            continue
        # Check if the topic matches
        for topic in args.topic_list:
            if topic_bag == topic:
                # Calculate hz
                freq_analyzer[topic].addTimestamp(msg.header.stamp.to_sec())

    # Plot the result
    cpu_analyzer.printResult()
    for topic in args.topic_list:
        freq_analyzer[topic].printResult()

    plt.show()
