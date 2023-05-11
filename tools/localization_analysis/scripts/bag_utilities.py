#!/usr/bin/python
#
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
"""
Utility functions for interacting with ros bags.
"""

import rosbag

# Load poses from a bag file for a set of poses with desired topics.
# Start at the provided bag start time.
def load_pose_msgs(vec_of_poses, bag, bag_start_time):
    topics = [poses.topic for poses in vec_of_poses]
    for topic, msg, t in bag.read_messages(topics):
        for poses in vec_of_poses:
            if poses.topic == topic:
                poses.add_msg(msg, msg.header.stamp, bag_start_time)
                break

# Load odometry poses from a bag file for a set of odometry poses with desired topics.
# Start at the provided bag start time.
def load_odometry_msgs(vec_of_poses, bag, bag_start_time):
    topics = [poses.topic for poses in vec_of_poses]
    for topic, msg, t in bag.read_messages(topics):
        for poses in vec_of_poses:
            if poses.topic == topic:
                poses.add_msg_with_covariance(
                    msg.odometry.body_F_source_T_target,
                    msg.header.stamp,
                    bag_start_time,
                )
                break

# Loads covariances from a bag file given a set of covariance topics.
# Start at the provided bag start time.
def load_pose_with_cov_msgs(vec_of_poses, bag, bag_start_time):
    topics = [poses.topic for poses in vec_of_poses]
    for topic, msg, t in bag.read_messages(topics):
        for poses in vec_of_poses:
            if poses.topic == topic:
                poses.add_msg_with_covariance(
                    msg.pose, msg.header.stamp, bag_start_time
                )
                break

# Loads localization states from a bag file given a set of localization topics.
# Start at the provided bag start time.
def load_loc_state_msgs(vec_of_loc_states, bag, bag_start_time):
    topics = [loc_states.topic for loc_states in vec_of_loc_states]
    for topic, msg, t in bag.read_messages(topics):
        for loc_states in vec_of_loc_states:
            if loc_states.topic == topic:
                loc_states.add_loc_state(msg, bag_start_time)
                break

# Loads velocities from a bag file given a set of velocity topics.
# Start at the provided bag start time.
def load_velocity_msgs(velocities, bag, bag_start_time):
    topics = [velocities.topic]
    for topic, msg, t in bag.read_messages(topics):
        velocities.add_msg(msg, msg.header.stamp, bag_start_time)

# Returns if the topic is available in the bag file
def has_topic(bag, topic):
    topics = bag.get_type_and_topic_info().topics
    return topic in topics
