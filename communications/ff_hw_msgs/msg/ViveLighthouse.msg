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

string serial                         # Lighthouse serial number
uint8 id                              # Used in light correspondences
uint8 fw_version                      # Firmware revision
uint8 hw_version                      # Hardware revision
uint8 mode                            # Operating mode 'a', 'b' or 'c'
uint8 MODE_A = 0
uint8 MODE_B = 1
uint8 MODE_C = 2
uint8 fault_mask                      # Faults
ff_hw_msgs/ViveMotor[] motors         # Motor calibration info
geometry_msgs/Vector3 acceleration    # Acceleration in lighthouse frame
