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
stream=$2

if [ "$camera" = dock ]
then
  if [ "$stream" = true ]
  then 
    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: DOCK}, {data_type: 0, b: true}, {data_type: 5, s: 1024x768}, {data_type: 2, f: 10}, {data_type: 2, f: 0}]}'
  else
    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: DOCK}, {data_type: 0, b: false}, {data_type: 5, s: 1024x768}, {data_type: 2, f: 10}, {data_type: 2, f: 0}]}'
  fi
elif [ "$camera" = nav ]
then
  if [ "$stream" = true ]
  then 
    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: NAV}, {data_type: 0, b: true}, {data_type: 5, s: 640x480}, {data_type: 2, f: 10}, {data_type: 2, f: 0}]}'
  else
    rostopic pub --once /command ff_msgs/CommandStamped '{cmd_name: "setCamera", subsys_name: "Astrobee", args: [{data_type: 5, s: NAV}, {data_type: 0, b: false}, {data_type: 5, s: 640x480}, {data_type: 2, f: 10}, {data_type: 2, f: 0}]}'
  fi
fi

echo "All done"
