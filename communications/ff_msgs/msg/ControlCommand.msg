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
# two copies of a ControlState message, plus a header
# mode: the current mode we are in. only two states defined so far
# current: the current ControlState trajectory we should aim for
# next: the next ControlState trajectory, just in case

std_msgs/Header header
uint8 mode
uint8 MODE_IDLE = 0
uint8 MODE_STOP = 1
uint8 MODE_NOMINAL = 2
ff_msgs/ControlState current
ff_msgs/ControlState next
