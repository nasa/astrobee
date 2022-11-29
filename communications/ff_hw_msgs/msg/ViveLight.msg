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

std_msgs/Header header               # Header includes tracker serial and time
uint8 lighthouse            # Lighthouse ID
uint8 axis                  # Motor axis
uint8 AXIS_0 = 0
uint8 AXIS_1 = 1
ff_hw_msgs/VivePulse[] pulses # Pulse times
