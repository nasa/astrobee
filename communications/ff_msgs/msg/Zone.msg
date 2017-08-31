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
#
# This message defines a zone, such as a Keepin or Keeoput.

string name                   # Name of zone

# A name can refer to multiple zones. This is the index of the zone with respect
# to the zone name
int32 index

# Zone type
uint8 KEEPOUT = 0       # An area the freeflyer should stay out of
uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in 
uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter

uint8 type              # Whether the zone is a keepin, keepout, or clutter

geometry_msgs/Vector3 min   # One corner of the zone
geometry_msgs/Vector3 max   # The opposite corner of the zone
