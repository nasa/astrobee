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
# This message describes the MOBILITY FEEDBACK message

uint8 state
uint8 INITIALIZING        = 0
uint8 WAITING             = 1
uint8 EXEC_VALIDATING     = 2
uint8 EXEC_BOOTSTRAPPING  = 3
uint8 EXEC_PLANNING       = 4
uint8 EXEC_CONTROLLING    = 5
uint8 EXEC_REPLANNING     = 6
uint8 MOVE_PLANNING       = 7
uint8 MOVE_CONTROLLING    = 8
uint8 MOVE_REPLANNING     = 9
uint8 STOP_CONTROLLING    = 10
uint8 IDLE_CONTROLLING    = 11
float32 perc_complete
float32 secs_remaining
ff_msgs/ControlProgress control