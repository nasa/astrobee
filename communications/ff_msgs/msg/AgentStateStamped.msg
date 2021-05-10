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
# State of the Astrobee, based off of rapid::ext::astrobee::AgentState

# Header with timestamp
std_msgs/Header header

# Operating state of the Astrobee
ff_msgs/OpState operating_state

# Plan execution state. State is idle when there is no plan to be executed. Once
# a plan is uploaded, the state change to paused. Upon a run plan command, the
# state will change to executing. If a stop or pause plan command is received or
# a fault occurs, the state will be set to pause. Once the plan is completed,
# the state will go back to idle
ff_msgs/ExecState plan_execution_state

# Guest science state. If a primary guest science apk is started, the state
# will go from idle to executing. Once the primarty apk is finished, the state
# will go back to idle
ff_msgs/ExecState guest_science_state

# Mobility state of the Astrobee
ff_msgs/MobilityState mobility_state

# Proximity to the dock when docking and undocking. Proximity to a handrail when
# perching or unperching. 0 the rest of the time.
float32 proximity

# Name of profile configuration, i.e. Nominal, IgnoreObstacles, Faceforward,
# Quiet, etc. Profiles specify stuff like target velocity and acceleration,
# collision distance, etc.
string profile_name

#Defines GN&C gains, hard limits, tolerances, etc.
string flight_mode

# Maximum linear velocity to target while translating
float32 target_linear_velocity
# Maximum linear acceleration to target while translating
float32 target_linear_accel
# Maximum angular velocity to target while rotating
float32 target_angular_velocity
# Maximum angular acceleration to target while rotating
float32 target_angular_accel
# Minimum distance margin to maintain away from obstacles
float32 collision_distance

# Specifies whether the Astrobee is allowed to fly blind i.e. not faceforward
bool holonomic_enabled

# Specifies whether the Astrobee is checking for obstacles
bool check_obstacles

# Specifies whether the Astrobee is checking the keepin and keepout zones
bool check_zones

# Specifies whether the Astrobee is allowed to auto return. Please note,
# Astrobee will only use this flags when deciding if it should auto return. If
# the astrobee gets a command to auto return from the operator or guest science,
# it will auto return without checking this flag
bool auto_return_enabled

# Specifies whether timesync is enabled. Hopefully this will be used to
# synchronize movements between Astrobees.
bool time_sync_enabled

# Specifies whether the choreographer should execute a segment immediately or
# based on the time stamp in the segement
bool immediate_enabled

# Specifies the current planner being used
string planner

# Specifies whether re-planning is allowed
bool replanning_enabled

# Specifies the current world being used
string world

# Number of seconds since Unix Epoch
uint32 boot_time
