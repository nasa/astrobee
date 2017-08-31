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
# Mobility states, based off the enumeration constants in
# rapid::ext::astrobee::AgentState
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 DRIFTING        = 0   # Astrobee's propulsion is off
uint8 STOPPING        = 1   # Astrobee is either stopping or stopped
uint8 FLYING          = 2   # Astrobee is flying
uint8 DOCKING         = 3   # Astrobee is either docking or undocking
uint8 PERCHING        = 4   # Astrobee is either perching or unperching

# Mobility state
uint8 state

# Specifies the progress of the action. For docking, this value can be N to -N
# where N through 1 specifies the progress of a docking action, 0 is docked, and
# -1 through -N specifies the process of an undocking action. For stopping, this
# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means
# the robot is stopped. For perching, this value can be N to -N where N through
# 1 specifies the progress of a perching action, 0 is perched, and -1 through
# -N specifies the process of an unperching action.
int32 sub_state
