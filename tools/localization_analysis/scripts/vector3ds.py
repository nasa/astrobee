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

import vector3d


class Vector3ds:
    def __init__(self):
        self.xs = []
        self.ys = []
        self.zs = []

    def add(self, x, y, z):
        self.xs.append(x)
        self.ys.append(y)
        self.zs.append(z)

    def add_vector3d(self, vector3d):
        self.xs.append(vector3d.x)
        self.ys.append(vector3d.y)
        self.zs.append(vector3d.z)

    def get_vector3d(self, index):
        return vector3d.Vector3d(self.xs[index], self.ys[index], self.zs[index])

    def get_numpy_vector(self, index):
        return np.array([self.xs[index], self.ys[index], self.zs[index]])
