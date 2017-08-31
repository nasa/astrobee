#!/bin/bash
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

dock=$1

# Position
x=-0.15
y=0.57
if [ "$dock" = true ]
then
  x=-0.54
  y=0.57
fi

# Orientation
#qx=0
#qy=0.0348994967
#qz=0
#qw=0.99939082701
qx=0.00277610053308
qy=0.0450436994433
qz=0.0410253852606
qw=0.998138487339


# Set to holonomic
rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setOperatingLimits", subsys_name: "Astrobee", args: [{data_type: 5, s: testing}, {data_type: 2, f: 0.20}, {data_type: 2, f: 0.01}, {data_type: 2, f: 0.09}, {data_type: 2, f: 0.07}, {data_type: 2, f: 0.0}, {data_type: 0, b: true}, {data_type: 2, f: 0.0}, {data_type: 0, b: false}]}'

# Action command
rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "simpleMove6DOF", subsys_name: "Astrobee", args: [{data_type: 5, s: world}, {data_type: 6, vec3d: ['$x', '$y', 0]}, {data_type: 6, vec3d: [0.10, 0.10, 0.10]}, {data_type: 7, mat33f: ['$qx', '$qy', '$qz', '$qw', 0, 0, 0, 0, 0]}]}'

echo "All done"
