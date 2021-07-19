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


class ImuMeasurements():

  def __init__(self):
    self.accelerations = vector3ds.Vector3ds()
    self.angular_velocities = vector3ds.Vector3ds()
    self.times = []

  def add_imu_measurement(self, msg):
    self.accelerations.add_vector3d(msg.linear_acceleration)
    self.angular_velocities.add_vector3d(msg.angular_velocity)
    self.times.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
