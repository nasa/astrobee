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

name=$1
nodelet_type=$2
manager_name=$3
bond_id=$4

if [[ $# -eq 1 ]]; then
  rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "loadNodelet", subsys_name: "Astrobee", args: [{data_type: 5, s: '"$name"'}]}'
elif [[ $# -eq 2 ]]; then
  rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "loadNodelet", subsys_name: "Astrobee", args: [{data_type: 5, s: '"$name"'}, {data_type: 5, s: '"$nodelet_type"'}]}'
elif [[ $# -eq 3 ]]; then
  rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "loadNodelet", subsys_name: "Astrobee", args: [{data_type: 5, s: '"$name"'}, {data_type: 5, s: '"$nodelet_type"'}, {data_type: 5, s: '"$manager_name"'}]}'
elif [[ $# -eq 4 ]]; then
  rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "loadNodelet", subsys_name: "Astrobee", args: [{data_type: 5, s: '"$name"'}, {data_type: 5, s: '"$nodelet_type"'}, {data_type: 5, s: '"$manager_name"'}, {data_type: 5, s: '"$bond_id"'}]}'
fi
