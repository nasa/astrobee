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
# This message describes the GNC CONTROL FEEDBACK message

std_msgs/Header header

# Are we waiting to start?
bool waiting_to_start

# Infomration about the previous, current and next setpoints
ff_msgs/ControlState prev_setpoint  # Previous setpoint
ff_msgs/ControlState goal_setpoint  # Where we should be
ff_msgs/ControlState next_setpoint  # Next setpoint

# Our current error
float32 error_position              # Position error
float32 error_attitude              # Attitude error
float32 error_velocity              # Velocity error
float32 error_omega                 # Omega error
float32 error_time_perc             # Time error

# The index of the prev_setpoint in the current segment
uint32 index
