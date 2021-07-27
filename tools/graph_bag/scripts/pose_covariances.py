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


class PoseCovariances:

  def __init__(self):
    self.position = vector3ds.Vector3ds()
    self.orientation = vector3ds.Vector3ds()

  # Assumes 6x6 covariance matrix stored in row major order
  # 0  1  2  3  4  5 
  # 6  7  8  9  10 11
  # 12 13 14 15 16 17 
  # 18 19 20 21 22 23
  # 24 25 26 27 28 29 
  # 30 31 32 33 34 35
  # Then the position submatrix is 
  # 0  1  2  
  # 6  7  8  
  # 12 13 14 
  # And the orientation submatrix is 
  # 21 22 23
  # 27 28 29 
  # 33 34 35
  def add(self, covariance_vector):
    x = covariance_vector[0]
    y = covariance_vector[7]
    z = covariance_vector[14]
    self.position.add(x,y,z)

    rot_x = covariance_vector[21]
    rot_y = covariance_vector[28]
    rot_z = covariance_vector[35]
    self.orientation.add(rot_x,rot_y,rot_z)
