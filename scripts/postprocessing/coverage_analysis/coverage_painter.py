#!/usr/bin/env python

""" Show spheres/markers representing where nav_cam was when it registered spheres
    representing Map Landmarks.
"""

import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import rospy
from tf.transformations import *
from visualization_msgs.msg import Marker, MarkerArray


class Coverage_Painter:
    def __init__(self):
        print("Coverage_Painter started")
        self.max_num_features = 0
        self.first_bin = 0
        self.second_bin = 0
        self.third_bin = 0
        self.fourth_bin = 0
        self.fifth_bin = 0

        self.counter = 0
        self.cube_size = 0.30  # Meters
        self.markerArray = MarkerArray()

    def is_robot_far_from_last_pose(self, last_pose, current_pose, min_dist):

        min_dist_squared = ((min_dist) ** 2) * 3  # cm

        error_squared = (
            (current_pose[0] - last_pose[0]) ** 2
            + (current_pose[1] - last_pose[1]) ** 2
            + (current_pose[2] - last_pose[2]) ** 2
        )

        # True if displacement larger than minimum distance, min_dist
        return error_squared > min_dist_squared

    def transform_body_to_gazebo(self, px, py, pz, qi, qj, qk, qw):
        # As defined in Astrobee repo: astrobee/config/robots/bumble.config
        pose_const = np.array(
            [0.0, 0.0, 0.0, 1.0]
        )  # Constant translation between Astrobee body and Gazebo frames
        quat_const = [
            -0.5,
            -0.5,
            0.5,
            0.5,
        ]  # Constant rotation between Astrobee body and Gazebo frames, (0, -90, 90) deg

        rot_const = quaternion_matrix(quat_const)
        rot_const[:, 3] = pose_const.T

        rot_in = quaternion_matrix([qi, qj, qk, qw])
        rot_in[:, 3] = np.array([px, py, pz, 1.0]).T

        t_body = rot_in.dot(rot_const)

        quat_out = np.zeros((4, 4))
        quat_out[0:3, 0:3] = t_body[0:3, 0:3]
        quat_out[3, 3] = 1.0

        quat_out = quaternion_from_matrix(quat_out)
        pose_out = t_body[0:3:, 3]

        return [
            pose_out[0],
            pose_out[1],
            pose_out[2],
            quat_out[0],
            quat_out[1],
            quat_out[2],
            quat_out[3],
        ]

    # Use Astrobee body frame database
    # Write the file to visualize each of the robot's pose coverage
    def write_robot_coverage(self, fileIn):

        last_pose = [0.0, 0.0, 0.0]
        min_dist = (
            0.0  # Minimum number of cm away from each other to be painted (e.g. 0.1 m)
        )

        print("Reading: " + fileIn)
        with open(fileIn) as f_in:

            rospy.init_node("register")

            # print("Pose x y z has ML features")
            marker_id = 1
            # lines = f.readlines()
            for line in f_in:

                # Skip empty lines and those starting with comments
                if re.match("^\s*\n", line) or re.match("^\s*\#", line):
                    continue

                # Extract all the values separated by spaces
                vals = []
                for v in re.split("\s+", line):
                    if v == "":
                        continue
                    vals.append(v)

                marker_text = ""
                # First line corresponds to number of ML & camera pose: #ML x y z i j k w
                if len(vals) == 8:
                    robot_pose = [float(vals[1]), float(vals[2]), float(vals[3])]

                    if self.is_robot_far_from_last_pose(
                        last_pose, robot_pose, min_dist
                    ):
                        color_code = self.get_color_name(int(vals[0]))
                        robot_rot = [
                            float(vals[4]),
                            float(vals[5]),
                            float(vals[6]),
                            float(vals[7]),
                        ]
                        q = self.transform_body_to_gazebo(
                            robot_pose[0],
                            robot_pose[1],
                            robot_pose[2],
                            robot_rot[0],
                            robot_rot[1],
                            robot_rot[2],
                            robot_rot[3],
                        )

                        robot_rot = [q[3], q[4], q[5], q[6]]

                        marker = Marker()
                        marker.header.frame_id = "world"
                        marker.ns = "thick_traj"
                        marker.id = marker_id + 1
                        marker.type = marker.ARROW
                        marker.action = marker.ADD
                        marker.scale.x = self.cube_size / 1.8
                        marker.scale.y = self.cube_size / 2.8
                        marker.scale.z = 0.05
                        marker.color.a = 1.0
                        marker.color.r = color_code[0]
                        marker.color.g = color_code[1]
                        marker.color.b = color_code[2]
                        marker.pose.position.x = robot_pose[0]
                        marker.pose.position.y = robot_pose[1]
                        marker.pose.position.z = robot_pose[2]
                        marker.pose.orientation.x = robot_rot[0]
                        marker.pose.orientation.y = robot_rot[1]
                        marker.pose.orientation.z = robot_rot[2]
                        marker.pose.orientation.w = robot_rot[3]

                        self.animate_markers(marker)
                        last_pose = robot_pose

                else:
                    # print("Skipping invalid line")
                    continue

    # Use Astrobee body frame database
    def write_map_coverage(self, fileIn):

        rospy.init_node("register")
        marker = Marker()

        print("Reading: " + fileIn)
        with open(fileIn) as f_in:

            marker_id = 1
            for line in f_in:  # lines:

                # Skip empty lines and those starting with comments
                # if re.match("^\s*\n", line) or re.match("^\s*\#", line):
                if re.match("^\s*\n", line):
                    continue

                # Check if comments has string "Wall"
                if re.match("^\s*\#", line):
                    vals = []
                    for v in re.split("\s+", line):
                        if v == "":
                            continue
                        vals.append(v)

                    if vals[1] == "Wall":
                        marker_scale = [
                            float(vals[3]),
                            float(vals[3]),
                            float(vals[3]),
                        ]  # Grab the grid_size for this wall

                else:
                    # Extract all the values separated by spaces
                    vals = []
                    for v in re.split("\s+", line):
                        if v == "":
                            continue
                        vals.append(v)

                    if len(vals) == 4:
                        color_code = self.get_color_name(int(vals[0]))
                        robot_pose = [float(vals[1]), float(vals[2]), float(vals[3])]
                        robot_rot = [0.0, 0.0, 0.0, 1.0]

                        marker = Marker()
                        marker.header.frame_id = "world"
                        marker.ns = "cube_walls"
                        marker.id = marker_id + 1
                        marker.type = marker.CUBE
                        marker.action = marker.ADD
                        marker.scale.x = marker_scale[0]
                        marker.scale.y = marker_scale[1]
                        marker.scale.z = marker_scale[2]
                        marker.color.a = 1.0
                        marker.color.r = color_code[0]
                        marker.color.g = color_code[1]
                        marker.color.b = color_code[2]
                        marker.pose.position.x = robot_pose[0]
                        marker.pose.position.y = robot_pose[1]
                        marker.pose.position.z = robot_pose[2]
                        marker.pose.orientation.x = robot_rot[0]
                        marker.pose.orientation.y = robot_rot[1]
                        marker.pose.orientation.z = robot_rot[2]
                        marker.pose.orientation.w = robot_rot[3]

                        self.animate_markers(marker)

                    else:
                        # print("Skipping invalid line")
                        continue

    def animate_markers(self, marker):
        # Publish
        topic = "/loc/registration"
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)

        self.markerArray.markers.append(marker)

        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        publisher.publish(self.markerArray)
        rospy.sleep(0.01)  # Wait these many seconds to publish the next marker

    def print_stats(self):
        print("Resulting Stats: ")
        # To get some stats
        total_entries = (
            self.first_bin
            + self.second_bin
            + self.third_bin
            + self.fourth_bin
            + self.fifth_bin
        )
        one = (self.first_bin) * 100.0 / total_entries
        two = (self.second_bin) * 100.0 / total_entries
        three = (self.third_bin) * 100.0 / total_entries
        four = (self.fourth_bin) * 100.0 / total_entries
        five = (self.fifth_bin) * 100.0 / total_entries
        print(
            "1stQ: {:d}, 2ndQ: {:d}, 3rdQ: {:d}, 4thQ: {:d}, 5thQ: {:d}, AllQ: {:d}".format(
                self.first_bin,
                self.second_bin,
                self.third_bin,
                self.fourth_bin,
                self.fifth_bin,
                total_entries,
            )
        )
        print(
            "1st%: {:.2f}, 2nd%: {:.2f}, 3rd%: {:.2f}, 4th%: {:.2f}, 5th%: {:.2f}".format(
                one, two, three, four, five
            )
        )

    def color_converter(self, color_name):
        # https://www.geeksforgeeks.org/switch-case-in-python-replacement/
        converter = {
            "red": [1.0, 0.0, 0.0],
            "orange": [1.0, 0.5, 0.0],
            "green": [0.0, 1.0, 0.0],
            "blue": [0.0, 0.0, 1.0],
            "yellow": [1.0, 1.0, 0.0],
            "white": [1.0, 1.0, 1.0],
        }
        return converter.get(color_name, [1.0, 0.0, 0.0])  # default: red

    # Get the max number of features from file
    def coverage_max_num_features(self, fileIn):

        with open(fileIn) as f_in:
            for line in f_in:
                # Skip empty lines and those starting with comments
                if re.match("^\s*\n", line) or re.match("^\s*\#", line):
                    continue

                # Extract all the values separated by spaces
                vals = []
                for v in re.split("\s+", line):
                    if v == "":
                        continue
                    vals.append(v)

                if len(vals) == 4:
                    if int(vals[0]) > self.max_num_features:
                        self.max_num_features = int(vals[0])
            print("Max num features: {:d}".format(self.max_num_features))

    # Get the color name based on how many features found in cube
    def get_color_name(self, num_features):

        # If ML number based visualization is desired
        if num_features == 0:
            self.first_bin += 1
            return self.color_converter("red")
        elif num_features >= 1 and num_features <= 10:
            self.second_bin += 1
            return self.color_converter("orange")
        elif num_features >= 11 and num_features <= 20:
            self.third_bin += 1
            return self.color_converter("yellow")
        elif num_features >= 21 and num_features <= 40:
            self.fourth_bin += 1
            return self.color_converter("green")
        elif num_features >= 40:
            self.fifth_bin += 1
            return self.color_converter("blue")


# The main program
if __name__ == "__main__":

    try:
        if len(sys.argv) < 3:
            print("Usage: " + sys.argv[0] + "<database_file> <robot_or_map_coverage>")
            print("       <database_file>: 'dir/to/my-database-file.csv' ")
            print("       <robot_or_map_coverage>: 'robot_coverage' | 'map_coverage'")
            sys.exit(1)

        fileIn = sys.argv[1]

        obj = Coverage_Painter()

        obj.coverage_max_num_features(fileIn)

        if sys.argv[2] == "map_coverage":
            obj.write_map_coverage(fileIn)

        elif sys.argv[2] == "robot_coverage":
            obj.write_robot_coverage(fileIn)

        obj.print_stats()

    except KeyboardInterrupt:
        print("\n <-CTRL-C EXIT: USER manually exited!->")
        sys.exit(0)
