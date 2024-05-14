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

import numpy as np

from vector3d_plotter import Vector3dPlotter


# Clip rotation angles so they stay between 0 and 360
def unwrap_in_degrees(angles):
    return np.rad2deg(np.unwrap(np.deg2rad(angles)))


# Class for plotting YPR orientation values on the same and separate plots.
class OrientationPlotter(Vector3dPlotter):
    def __init__(
        self,
        name,
        times,
        y_vals,
        p_vals,
        r_vals,
        colors=["r", "b", "g"],
        linestyle="-",
        linewidth=1,
        marker=None,
        markeredgewidth=None,
        markersize=1,
    ):
        super(OrientationPlotter, self).__init__(
            name,
            times,
            unwrap_in_degrees(y_vals),
            unwrap_in_degrees(p_vals),
            unwrap_in_degrees(r_vals),
            ["Orientation (Yaw)", "Orientation (Roll)", "Orientation (Pitch)"],
            colors,
            linestyle,
            linewidth,
            marker,
            markeredgewidth,
            markersize,
        )
