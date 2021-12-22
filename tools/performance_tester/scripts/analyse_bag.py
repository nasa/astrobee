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
default_topics = ["/loc/ml/features", "/hw/imu", "/gnc/ekf", "/gnc/ctl/command"]


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

    def printResult(self, i, ax):
        # Plot the frequency over time
        xpoints = np.array(self.time) - self.time[0]
        ypoints = np.array(self.frequency)

        ax[i, 0].plot(xpoints, ypoints)
        ax[i, 0].set_xlabel("Time(s)")
        ax[i, 0].set_ylabel("Frequency (Hz)")
        ax[i, 0].set_title(self.name)

        # Print the histogram with percentage y-axis
        num_bins = 100
        heights, bins = np.histogram(self.t_diff, num_bins)
        percent = [float(i) / sum(heights) * 100 for i in heights]
        ax[i, 1].bar(bins[:-1], percent, width=bins[1] - bins[0], align="edge")
        ax[i, 1].set_xlabel("Time(s)")
        ax[i, 1].set_ylabel("Percentage(%)")
        ax[i, 1].set_title(self.name + " Time between messages histogram")

    @classmethod
    def initDictFromTopics(cls, topic_list):
        return dict(
            (topic, cls(topic, i, len(topic_list)))
            for i, topic in enumerate(topic_list)
        )

    @classmethod
    def plotResults(cls, topics):
        # Start figure
        fig, ax = plt.subplots(len(topics), 2, figsize=(16, 16))
        plt.subplots_adjust(
            left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.35
        )
        for i, key in enumerate(topics.keys()):
            topics[key].printResult(i - 1, ax)

        plt.show()


class CpuUsage:
    def __init__(self):
        self.cpu_topic = "/mgt/cpu_monitor/state"

        # Total CPU usage
        self.mlp_cpu_loads = [[], [], [], []]
        self.mlp_time = []
        self.llp_cpu_loads = [[], []]
        self.llp_time = []

        # Node CPU usage
        self.llp_nodes = {}
        self.llp_nodes_time = []
        self.mlp_nodes = {}
        self.mlp_nodes_time = []

    def addCpuMessage(self, topic, msg):
        # Check if the topic is the one we're interested in
        if topic != self.cpu_topic:
            return

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

            # Save nodes info
            append_time_llp = False
            for node in msg.load_nodes:
                append_time_llp = True
                if node.name in self.llp_nodes:
                    self.llp_nodes[node.name].append(node.load)
                else:
                    self.llp_nodes[node.name] = [node.load]
            if append_time_llp:
                self.llp_nodes_time.append(msg.header.stamp.to_sec())

        if msg.name == "mlp":
            self.mlp_time.append(msg.header.stamp.to_sec())
            for i in range(len(msg.cpus)):
                self.mlp_cpu_loads[i].append(msg.cpus[i].loads[total_cpu])

            # Save nodes info
            append_time_mlp = False
            for node in msg.load_nodes:
                append_time_mlp = True
                if node.name in self.mlp_nodes:
                    self.mlp_nodes[node.name].append(node.load)
                else:
                    self.mlp_nodes[node.name] = [node.load]
            if append_time_mlp:
                self.mlp_nodes_time.append(msg.header.stamp.to_sec())

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
        ax[0, 0].set_ylim([0, 100])

        # Plot MLP CPU usage
        xpoints = np.array(self.mlp_time) - self.llp_time[0]
        for cpu in range(len(self.mlp_cpu_loads)):
            ypoints = np.array(self.mlp_cpu_loads[cpu])
            ax[0, 1].plot(xpoints, ypoints, label="cpu" + str(cpu))
        ax[0, 1].set_xlabel("Time(s)")
        ax[0, 1].set_ylabel("Load")
        ax[0, 1].set_title("MLP CPU loads")
        ax[0, 1].legend()
        ax[0, 1].set_ylim([0, 100])

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
        ax[2, 0].set_title("Node CPU commulative loads")
        ax[2, 0].legend()

        ax[2, 1].stackplot(
            np.array(self.mlp_nodes_time) - self.mlp_nodes_time[0],
            self.mlp_nodes.values(),
            labels=self.mlp_nodes.keys(),
            alpha=0.8,
        )
        ax[2, 1].set_xlabel("Time(s)")
        ax[2, 1].set_ylabel("Load")
        ax[2, 1].set_title("Node CPU commulative loads")
        ax[2, 1].legend()


class MemUsage:
    def __init__(self):
        self.mem_topic = "/mgt/mem_monitor/state"
        # Analyse total memory usage
        self.llp_mem_load = []
        self.llp_time = []
        self.mlp_mem_load = []
        self.mlp_time = []
        # Analyse individual node memory usage
        self.llp_nodes = {}
        self.llp_nodes_time = []
        self.mlp_nodes = {}
        self.mlp_nodes_time = []

        self.llp_init = False
        self.mlp_init = False

    def addMemMessage(self, topic, msg):
        # Check if the topic is the one we're interested in
        if topic != self.mem_topic:
            return

        # Save ram values
        if msg.name == "llp":
            if self.llp_init == False:
                self.llp_ram_total = msg.ram_total
            self.llp_mem_load.append(msg.ram_used)
            self.llp_time.append(msg.header.stamp.to_sec())

            append_time_llp = False
            for node in msg.nodes:
                append_time_llp = True
                if node.name in self.llp_nodes:
                    self.llp_nodes[node.name].append(node.ram_perc)
                else:
                    self.llp_nodes[node.name] = [node.ram_perc]
            if append_time_llp:
                self.llp_nodes_time.append(msg.header.stamp.to_sec())

        if msg.name == "mlp":
            if self.mlp_init == False:
                self.mlp_ram_total = msg.ram_total
            self.mlp_mem_load.append(msg.ram_used)
            self.mlp_time.append(msg.header.stamp.to_sec())

            append_time_mlp = False
            for node in msg.nodes:
                append_time_mlp = True
                if node.name in self.mlp_nodes:
                    self.mlp_nodes[node.name].append(node.ram_perc)
                else:
                    self.mlp_nodes[node.name] = [node.ram_perc]
            if append_time_mlp:
                self.mlp_nodes_time.append(msg.header.stamp.to_sec())

    def printResult(self):
        fig, ax = plt.subplots(2, 2, figsize=(16, 16))

        # Plot LLP CPU usage
        xpoints = np.array(self.llp_time) - self.llp_time[0]
        ypoints = 100 * np.array(self.llp_mem_load) / self.llp_ram_total
        ax[0, 0].plot(xpoints, ypoints)
        ax[0, 0].set_xlabel("Time(s)")
        ax[0, 0].set_ylabel("Load")
        ax[0, 0].set_title("LLP Memory RAM loads")
        # ax[0, 0].legend()
        ax[0, 0].set_ylim([0, 100])

        # Plot MLP CPU usage
        xpoints = np.array(self.mlp_time) - self.llp_time[0]
        ypoints = 100 * np.array(self.mlp_mem_load) / self.mlp_ram_total
        ax[0, 1].plot(xpoints, ypoints)
        ax[0, 1].set_xlabel("Time(s)")
        ax[0, 1].set_ylabel("Load")
        ax[0, 1].set_title("MLP Memory RAM loads")
        # ax[0, 1].legend()
        ax[0, 1].set_ylim([0, 100])

        # Plot nodes usage
        # xpoints = np.array(self.llp_nodes_time) - self.llp_nodes_time[0]
        # for llp_node in self.llp_nodes:
        #     ax[1, 0].plot(xpoints, self.llp_nodes[llp_node], label=llp_node, alpha=0.8)
        # ax[1, 0].set_xlabel("Time(s)")
        # ax[1, 0].set_ylabel("Load")
        # ax[1, 0].set_title("Node Memory RAM loads")
        # ax[1, 0].legend()

        # xpoints = np.array(self.mlp_nodes_time) - self.mlp_nodes_time[0]
        # for mlp_node in self.mlp_nodes:
        #     ax[1, 1].plot(xpoints, self.mlp_nodes[mlp_node], label=mlp_node, alpha=0.8)
        # ax[1, 1].set_xlabel("Time(s)")
        # ax[1, 1].set_ylabel("Load")
        # ax[1, 1].set_title("Node Memory RAM loads")
        # ax[1, 1].legend()

        # Plot nodes usage cummulative
        ax[1, 0].stackplot(
            np.array(self.llp_nodes_time) - self.llp_nodes_time[0],
            self.llp_nodes.values(),
            labels=self.llp_nodes.keys(),
            alpha=0.8,
        )
        ax[1, 0].set_xlabel("Time(s)")
        ax[1, 0].set_ylabel("Load")
        ax[1, 0].set_title("Node Memory RAM commulative loads")
        ax[1, 0].legend()

        ax[1, 1].stackplot(
            np.array(self.mlp_nodes_time) - self.mlp_nodes_time[0],
            self.mlp_nodes.values(),
            labels=self.mlp_nodes.keys(),
            alpha=0.8,
        )
        ax[1, 1].set_xlabel("Time(s)")
        ax[1, 1].set_ylabel("Load")
        ax[1, 1].set_title("Node CPU commulative loads")
        ax[1, 1].legend()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-name", default="")
    parser.add_argument("-n", "--topic-list", nargs="+", default=default_topics)

    args = parser.parse_args()

    # Create topic dictionary
    freq_analyzer = Topic.initDictFromTopics(args.topic_list)
    # Analyse resource loads
    cpu_analyzer = CpuUsage()
    mem_analyzer = MemUsage()

    # Go through the messages in the bag file
    for topic_bag, msg, t in rosbag.Bag(args.bag_name).read_messages():
        # Add message to cpu analyser
        cpu_analyzer.addCpuMessage(topic_bag, msg)
        # Add message to mem analyser
        mem_analyzer.addMemMessage(topic_bag, msg)
        # Check if the topic matches
        for topic in args.topic_list:
            if topic_bag == topic:
                # Calculate hz
                freq_analyzer[topic].addTimestamp(msg.header.stamp.to_sec())

    # Plot the result
    cpu_analyzer.printResult()
    mem_analyzer.printResult()
    Topic.plotResults(freq_analyzer)
