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
# The state of the arm behavior

# Header with timestamp
std_msgs/Header header

# Tee current state
int8 state                         # Current state
int8 INITIALIZING        = 0       # Waiting on child services, actions, etc.
int8 IDLE                = 1
int8 MOVING              = 2
int8 DOCKING             = 3


# A human readble version of the (event) -> [state] transition
string fsm_event
string fsm_state
