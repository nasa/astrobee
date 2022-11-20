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
# This message captures all information in a flight mode

std_msgs/Header header                     # Metadata

string name                       # Name of the flight mode

bool control_enabled              # Is control enabled?

# Tolerances (all in SI units)
float32 tolerance_pos_endpoint    # Endpoint position tolerance in m
float32 tolerance_pos             # Position tolerance in m
float32 tolerance_vel             # Velocity tolerance in m/s
float32 tolerance_att             # Attitude tolerance in rads
float32 tolerance_omega           # Angular acceleration tolerance in rad/s
float32 tolerance_time            # Acceptable lag betwee TX and RX of control

# Controller gains
geometry_msgs/Vector3 att_kp      # Positional proportional constant
geometry_msgs/Vector3 att_ki      # Positional integrative constant
geometry_msgs/Vector3 omega_kd    # Attidue derivative constant
geometry_msgs/Vector3 pos_kp      # Positional proportional contant
geometry_msgs/Vector3 pos_ki      # Positional integrative constant
geometry_msgs/Vector3 vel_kd      # Positional derivative constant

# Hard limit on planning
float32 hard_limit_vel            # Position tolerance in m/s
float32 hard_limit_accel          # Position tolerance in m/s^2
float32 hard_limit_omega          # Position tolerance in rads/s
float32 hard_limit_alpha          # Position tolerance in rads/s^2

# Impeller speed
uint8 speed                       # Current speed gain
uint8 SPEED_MIN        = 0        # Min acceptable gain
uint8 SPEED_OFF        = 0        # Blowers off
uint8 SPEED_QUIET      = 1        # Quiet mode
uint8 SPEED_NOMINAL    = 2        # Nomainal mode
uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode
uint8 SPEED_MAX        = 3        # Max acceptable gain
