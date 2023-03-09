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

import numpy as np
import scipy.spatial.transform

import orientations
import pose
import pose_covariances
import vector3ds


class Poses(object):
    def __init__(self, pose_type, topic):
        self.positions = vector3ds.Vector3ds()
        self.orientations = orientations.Orientations()
        self.covariances = pose_covariances.PoseCovariances()
        self.times = []
        self.pose_type = pose_type
        self.topic = topic

    def init_from_poses(self, poses_list, times):
        for pose in poses_list:
            self.add_pose(pose)
        self.times = times

    def add_pose(self, pose):
        self.add_orientation_and_position(
            pose.orientation, pose.position[0], pose.position[1], pose.position[2]
        )

    def add_orientation_and_position(self, orientation, x, y, z):
        self.positions.add(x, y, z)
        euler_angles = orientation.as_euler("ZYX", degrees=True)
        self.orientations.add(euler_angles[0], euler_angles[1], euler_angles[2])

    def add_covariance_msg(self, covariance):
        self.covariances.add(covariance)

    def add_pose_msg(self, pose_msg, timestamp, bag_start_time=0):
        orientation = scipy.spatial.transform.Rotation.from_quat(
            [
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ]
        )
        self.add_orientation_and_position(
            orientation, pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
        )
        self.times.append(timestamp.secs + 1e-9 * timestamp.nsecs - bag_start_time)

    def add_msg_with_covariance(self, msg, timestamp, bag_start_time=0):
        self.add_pose_msg(msg.pose, timestamp, bag_start_time)
        self.add_covariance_msg(msg.covariance)

    def add_msg(self, msg, timestamp, bag_start_time=0):
        self.add_pose_msg(msg.pose, timestamp, bag_start_time)

    def position_vector(self, index):
        return [
            self.positions.xs[index],
            self.positions.ys[index],
            self.positions.zs[index],
        ]

    def pose(self, index):
        return pose.Pose(
            self.orientations.get_rotation(index), np.array(self.position_vector(index))
        )
