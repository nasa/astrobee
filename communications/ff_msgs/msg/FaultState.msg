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
# Fault state message used to alert the ground of the current faults. It is also
# used to express to the executive that a fault has occurred that indirectly
# affects the motion of the robot.

std_msgs/Header header

# Not sent to the ground, only used by the executive to determine what commands
# to accept.
uint8 state
# System starting up
uint8 STARTING_UP           = 0
# No faults are occurring in system
uint8 FUNCTIONAL            = 1
# Faults are occurring in the system which may or may not leave the robot
# functional
uint8 FAULT                 = 2
# A fault has occurred that indirectly affects the motion of the robot
uint8 BLOCKED               = 3
# Recovering from nodes dying on startup
uint8 RELOADING_NODELETS    = 4

# A human readable version of the state - only really used for when nodes die on
# startup and need to be restarted.
string hr_state

# Faults occurring in the astrobee system, can only send 32 faults down
ff_msgs/Fault[] faults
