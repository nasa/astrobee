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

import poses
import vector3ds


class LocStates(poses.Poses):

  def __init__(self, loc_type, topic):
    super(LocStates, self).__init__(loc_type, topic)
    self.of_counts = []
    self.vl_counts = []
    self.accelerations = vector3ds.Vector3ds()
    self.velocities = vector3ds.Vector3ds()
    self.angular_velocities = vector3ds.Vector3ds()
    self.accelerometer_biases = vector3ds.Vector3ds()
    self.gyro_biases = vector3ds.Vector3ds()

  def add_loc_state(self, msg):
    self.add_pose(msg.pose, msg.header.stamp)
    self.of_counts.append(msg.of_count)
    self.vl_counts.append(msg.ml_count)
    self.accelerations.add_vector3d(msg.accel)
    self.velocities.add_vector3d(msg.velocity)
    self.angular_velocities.add_vector3d(msg.omega)
    self.accelerometer_biases.add_vector3d(msg.accel_bias)
    self.gyro_biases.add_vector3d(msg.gyro_bias)
