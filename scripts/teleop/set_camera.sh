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

camera=$1
mode=$2
res=$3

if [ "$camera" = dock ]
then
  if [ "$mode" = record ]
  then
    if [ "$res" = low ]
    then
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Dock}, {data_type: 5, s: Recording}, {data_type: 5, s: 640x480}, {data_type: 2, f: 2}, {data_type: 2, f: 0}]}'
    else
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Dock}, {data_type: 5, s: Recording}, {data_type: 5, s: 1280x960}, {data_type: 2, f: 5}, {data_type: 2, f: 0}]}'
    fi
  elif [ "$mode" = stream ]
  then
    if [ "$res" = low ]
    then
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Dock}, {data_type: 5, s: Streaming}, {data_type: 5, s: 640x480}, {data_type: 2, f: 2}, {data_type: 2, f: 0}]}'
    else
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Dock}, {data_type: 5, s: Streaming}, {data_type: 5, s: 1280x960}, {data_type: 2, f: 5}, {data_type: 2, f: 0}]}'
    fi
  else
    if [ "$res" = low ]
    then
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Dock}, {data_type: 5, s: Both}, {data_type: 5, s: 640x480}, {data_type: 2, f: 2}, {data_type: 2, f: 0}]}'
    else
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Dock}, {data_type: 5, s: Both}, {data_type: 5, s: 1280x960}, {data_type: 2, f: 5}, {data_type: 2, f: 0}]}'
    fi
  fi
elif [ "$camera" = nav ]
then
  if [ "$mode" = record ]
  then
    if [ "$res" = low ]
    then
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Navigation}, {data_type: 5, s: Recording}, {data_type: 5, s: 640x480}, {data_type: 2, f: 2}, {data_type: 2, f: 0}]}'
    else
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Navigation}, {data_type: 5, s: Recording}, {data_type: 5, s: 1280x960}, {data_type: 2, f: 5}, {data_type: 2, f: 0}]}'
    fi
  elif [ "$mode" = stream ]
  then
    if [ "$res" = low ]
    then
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Navigation}, {data_type: 5, s: Streaming}, {data_type: 5, s: 640x480}, {data_type: 2, f: 2}, {data_type: 2, f: 0}]}'
    else
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Navigation}, {data_type: 5, s: Streaming}, {data_type: 5, s: 1280x960}, {data_type: 2, f: 5}, {data_type: 2, f: 0}]}'
    fi
  else
    if [ "$res" = low ]
    then
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Navigation}, {data_type: 5, s: Both}, {data_type: 5, s: 640x480}, {data_type: 2, f: 2}, {data_type: 2, f: 0}]}'
    else
      rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: Navigation}, {data_type: 5, s: Both}, {data_type: 5, s: 1280x960}, {data_type: 2, f: 5}, {data_type: 2, f: 0}]}'
    fi
  fi
elif [ "$camera" = sci ]
then
  echo "Sci cam not added to script yet!!!"
fi

echo "All done"
