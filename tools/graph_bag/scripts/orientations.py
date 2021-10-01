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
import scipy.spatial.transform


class Orientations:
    def __init__(self):
        self.yaws = []
        self.pitches = []
        self.rolls = []

    def get_euler(self, index):
        return [self.yaws[index], self.pitches[index], self.rolls[index]]

    def get_rotation(self, index):
        return scipy.spatial.transform.Rotation.from_euler(
            "ZYX", self.get_euler(index), degrees=True
        )

    def add_euler(self, euler_angles):
        self.add(euler_angles[0], euler_angles[1], euler_angles[2])

    def add(self, yaw, pitch, roll):
        self.yaws.append(yaw)
        self.pitches.append(pitch)
        self.rolls.append(roll)
