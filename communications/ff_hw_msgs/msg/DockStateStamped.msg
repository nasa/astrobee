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
# Message used to store dock state

std_msgs/Header header

# Whether astrobee is docked or not
int8 state

int8 ERROR    = -1
int8 UNKNOWN  = 0
int8 DOCKED   = 1
int8 UNDOCKED = 2

# Which bay we are docked in
uint8 bay

uint8 BAY_UNKNOWN = 0  # unknown bay or not docked
uint8 BAY_ONE     = 1  # bay one, closest to the control panel
uint8 BAY_TWO     = 2  # bay two, under the chargers
