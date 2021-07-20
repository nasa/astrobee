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

class Pose:

  def __init__(self, orientation, position):
    self.orientation = orientation
    self.position = position

  def __rmul__(self, pose_b):
    new_orientation = self.orientation*pose_b.orientation
    new_position = self.orientation.apply(pose_b.position) + self.position 
    return Pose(new_orientation, new_position) 
