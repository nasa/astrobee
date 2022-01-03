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

import orientations
import scipy.spatial.transform
import vector3ds

import scipy.spatial.transform
import vector3ds

class Poses(object):
    def __init__(self, pose_type, topic):
        self.positions = vector3ds.Vector3ds()
        self.orientations = orientations.Orientations()
        self.times = []
        self.pose_type = pose_type
        self.topic = topic

    def add_pose(self, pose_msg, timestamp, bag_start_time=0):
        self.positions.add(
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
        )
        euler_angles = scipy.spatial.transform.Rotation.from_quat(
            [
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ]
        ).as_euler("ZYX", degrees=True)
        self.orientations.add(euler_angles[0], euler_angles[1], euler_angles[2])
        self.times.append(timestamp.secs + 1e-9 * timestamp.nsecs - bag_start_time)

    def add_msg(self, msg, timestamp, bag_start_time=0):
        self.add_pose(msg.pose, timestamp, bag_start_time)

    def position_vector(self, index):
        return [
            self.positions.xs[index],
            self.positions.ys[index],
            self.positions.zs[index],
        ]
