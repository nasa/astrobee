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

power=""

if [ "$1" = on ]; then
  power="powerOnItem"
elif [ "$1" = off ]; then
  power="powerOffItem"
else
  echo "Arg 1 not recognized. Must be on or off."
  exit
fi

item=""

if [ "$2" = "laser" ]; then
  item="Laser Pointer"
elif [ "$2" = "pmc" ]; then
  item="PMC"
elif [ "$2" = "top_aft" ]; then
  item="Payload Top Aft"
elif [ "$2" = "bottom_aft" ]; then
  item="Payload Bottom Aft"
elif [ "$2" = "bottom_front" ]; then
  item="Payload Bottom Front"
else
  echo "Arg 2 not recognized. Item must be laser, pmc, top_aft, bottom_aft, or bottom_front." 
  exit
fi

cmd="{cmd_name: $power, subsys_name: \"Astrobee\", args: [{data_type: 5, s: $item}]}"

rostopic pub --once /command ff_msgs/CommandStamped "$cmd"

echo "All done"
