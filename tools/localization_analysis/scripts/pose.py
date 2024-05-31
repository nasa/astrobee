#!/usr/bin/python3
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

from position import Position


# Pose class that contains an orientation and position and supports pose multiplication.
class Pose(object):
    def __init__(self, orientation, position):
        self.orientation = orientation
        self.position = position

    # Right multiply the pose by pose_b and return the resulting pose.
    def __mul__(self, pose_b):
        new_orientation = self.orientation * pose_b.orientation
        new_position = Position(self.orientation.apply(pose_b.position) + self.position)
        return Pose(new_orientation, new_position)

    # Invert the pose
    def inverse(self):
        new_orientation = self.orientation.inv()
        new_position = Position(-1.0 * new_orientation.apply(self.position))
        return Pose(new_orientation, new_position)

    # Returns the orientation as ZYX euler angles (YPR).
    def euler_angles(self):
        return self.orientation.as_euler("ZYX", degrees=True)

    # Returns the position.
    def position(self):
        return self.position
