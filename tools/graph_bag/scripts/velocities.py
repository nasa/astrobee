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

import vector3ds


class Velocities(object):

  def __init__(self, velocity_type, topic):
    self.velocities = vector3ds.Vector3ds()
    self.times = []
    self.velocity_type = velocity_type
    self.topic = topic

  def add_velocity(self, velocity_msg, timestamp, bag_start_time=0):
    self.velocities.add(velocity_msg.x, velocity_msg.y, velocity_msg.z)
    self.times.append(timestamp.secs + 1e-9 * timestamp.nsecs - bag_start_time)

  def add_msg(self, msg, timestamp, bag_start_time=0):
    self.add_velocity(msg.vector, timestamp, bag_start_time)
